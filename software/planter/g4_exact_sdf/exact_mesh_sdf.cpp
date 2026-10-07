// G4 exact source-triangle signed-distance plugin for MuJoCo. It never writes
// actuators, state, forces or poses. It answers distance and gradient queries
// against the original closed triangle mesh of its geom and, only when
// g4_exact_mesh_sdf_set_collider(1) is called, also generates mesh/SDF
// contacts for its instances (see exact_mesh_collide).
//
// Config attributes: `file` (below) and `starts`, the Frank-Wolfe starts per
// mesh face used by the exact collider (default: sdf_initpoints).
//
// Two geometry modes:
//   * "source_double": the plugin config attribute `file` names the binary STL
//     the compiled mesh came from. Its float32 millimetre vertices are promoted
//     to double, scaled and moved by the compiler's recorded mesh transform in
//     double, then checked face by face against the compiled float32 vertices.
//     Initialization fails closed if any face deviates by more than 50 nm.
//   * "compiled_float32": no `file` attribute. The compiled float32 vertices
//     are used directly, so the field inherits float32 rounding (a few nm).
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using V = std::array<double, 3>;
V add(V a, V b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
V sub(V a, V b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
V mul(V a, double t) { return {a[0]*t, a[1]*t, a[2]*t}; }
double dot(V a, V b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
V cross(V a, V b) { return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]}; }
double norm(V a) { return std::sqrt(dot(a, a)); }
V unit(V a) { double n = norm(a); if (!(n > 0)) throw std::runtime_error("zero normal"); return mul(a, 1/n); }

struct Closest { V p, bary; };
// Closest point on a triangle by plane projection plus the three edge
// candidates. Unlike the barycentric region tests this stays well conditioned
// on sliver triangles: the projection and the edge parameters are each
// computed from one well-scaled dot product, and a misclassified inside test
// on a sliver only swaps in an edge candidate of nearly equal distance.
Closest closest(V p, V a, V b, V c) {
  V ab = sub(b, a), ac = sub(c, a), n = cross(ab, ac);
  double n2 = dot(n, n);
  Closest best; double best2 = std::numeric_limits<double>::infinity();
  if (n2 > 0) {
    V proj = sub(p, mul(n, dot(sub(p, a), n)/n2)), ap = sub(proj, a);
    double beta = dot(cross(ap, ac), n)/n2, gamma = dot(cross(ab, ap), n)/n2;
    if (beta >= 0 && gamma >= 0 && beta+gamma <= 1) { best = {proj, {1-beta-gamma, beta, gamma}}; best2 = dot(sub(p, proj), sub(p, proj)); }
  }
  const V* corners[3] = {&a, &b, &c};
  for (int k = 0; k < 3; ++k) {
    V s = *corners[k], e = sub(*corners[(k+1)%3], s);
    double e2 = dot(e, e), t = e2 > 0 ? std::min(1., std::max(0., dot(sub(p, s), e)/e2)) : 0;
    V q = add(s, mul(e, t)), d = sub(p, q); double d2 = dot(d, d);
    if (d2 < best2) { V bary = {0, 0, 0}; bary[k] = 1-t; bary[(k+1)%3] = t; best = {q, bary}; best2 = d2; }
  }
  return best;
}

struct Node { V lo, hi; int left = -1, right = -1, begin = 0, end = 0; };
struct Edge { V normal = {0, 0, 0}; int count = 0, balance = 0; };

struct Mesh {
  std::vector<V> vertices, fn, vn;
  std::vector<std::array<int, 3>> faces;
  std::map<std::pair<int, int>, Edge> edges;
  std::vector<int> order;
  std::vector<Node> nodes;
  std::string mode;
  int starts = 0;  // Frank-Wolfe starts per face for the exact collider; 0: sdf_initpoints
  double compiled_deviation = 0;  // metres, source_double mode only

  Mesh(const mjModel* m, int instance) {
    int gid = -1;
    for (int g = 0; g < m->ngeom; ++g) if (m->geom_plugin[g] == instance) {
      if (gid != -1) throw std::runtime_error("one mesh geom per plugin instance required");
      gid = g;
    }
    if (gid < 0 || m->geom_type[gid] != mjGEOM_SDF) throw std::runtime_error("explicit mesh SDF geom required");
    int mid = m->geom_dataid[gid];
    if (mid < 0) throw std::runtime_error("explicit source mesh required");
    const char* file = mj_getPluginConfig(m, instance, "file");
    const char* fw = mj_getPluginConfig(m, instance, "starts");
    if (fw && fw[0]) {
      char* end = nullptr; long n = std::strtol(fw, &end, 10);
      if (*end || n < 1 || n > 1000) throw std::runtime_error("starts must be an integer in [1, 1000]");
      starts = int(n);
    }
    if (file && file[0]) { load_source(file, m, mid); mode = "source_double"; }
    else { load_compiled(m, mid); mode = "compiled_float32"; }
    finish();
  }

  void load_compiled(const mjModel* m, int mid) {
    int nv = m->mesh_vertnum[mid], nf = m->mesh_facenum[mid];
    vertices.resize(nv); faces.resize(nf);
    for (int i = 0; i < nv; ++i) for (int k = 0; k < 3; ++k) vertices[i][k] = m->mesh_vert[3*(m->mesh_vertadr[mid]+i)+k];
    for (int i = 0; i < nf; ++i) for (int k = 0; k < 3; ++k) faces[i][k] = m->mesh_face[3*(m->mesh_faceadr[mid]+i)+k];
  }

  void load_source(const char* path, const mjModel* m, int mid) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) throw std::runtime_error(std::string("cannot open source STL: ")+path);
    std::vector<unsigned char> bytes;
    unsigned char buffer[65536]; size_t got;
    while ((got = std::fread(buffer, 1, sizeof buffer, f)) > 0) bytes.insert(bytes.end(), buffer, buffer+got);
    std::fclose(f);
    if (bytes.size() < 84) throw std::runtime_error("source STL too short");
    uint32_t count; std::memcpy(&count, bytes.data()+80, 4);
    if (bytes.size() != 84+50ull*count) throw std::runtime_error("source STL size does not match binary triangle count");
    std::map<std::array<float, 3>, int> index;
    for (uint32_t t = 0; t < count; ++t) {
      std::array<int, 3> face;
      for (int k = 0; k < 3; ++k) {
        std::array<float, 3> v;
        std::memcpy(v.data(), bytes.data()+84+50ull*t+12+12*k, 12);
        auto it = index.find(v);
        if (it == index.end()) { it = index.emplace(v, int(vertices.size())).first; vertices.push_back({v[0], v[1], v[2]}); }
        face[k] = it->second;
      }
      faces.push_back(face);
    }
    // Compiler transform in double: compiled = R(mesh_quat)^T (scale * v - mesh_pos).
    mjtNum R[9]; mju_quat2Mat(R, m->mesh_quat+4*mid);
    for (auto& v : vertices) {
      V s = {v[0]*m->mesh_scale[3*mid]-m->mesh_pos[3*mid], v[1]*m->mesh_scale[3*mid+1]-m->mesh_pos[3*mid+1], v[2]*m->mesh_scale[3*mid+2]-m->mesh_pos[3*mid+2]};
      v = {R[0]*s[0]+R[3]*s[1]+R[6]*s[2], R[1]*s[0]+R[4]*s[1]+R[7]*s[2], R[2]*s[0]+R[5]*s[1]+R[8]*s[2]};
    }
    // Fail closed unless the file reproduces the compiled triangles.
    int nv = m->mesh_vertnum[mid], nf = m->mesh_facenum[mid];
    if (int(vertices.size()) != nv || int(faces.size()) != nf) throw std::runtime_error("source STL vertex/face counts differ from compiled mesh");
    for (int i = 0; i < nf; ++i) {
      double best = std::numeric_limits<double>::infinity();
      for (int rot = 0; rot < 3; ++rot) {
        double worst = 0;
        for (int k = 0; k < 3; ++k) {
          const float* c = m->mesh_vert+3*(m->mesh_vertadr[mid]+m->mesh_face[3*(m->mesh_faceadr[mid]+i)+k]);
          V s = vertices[faces[i][(k+rot)%3]];
          for (int j = 0; j < 3; ++j) worst = std::max(worst, std::fabs(s[j]-double(c[j])));
        }
        best = std::min(best, worst);
      }
      compiled_deviation = std::max(compiled_deviation, best);
    }
    if (!(compiled_deviation <= 5e-8)) throw std::runtime_error("source STL does not reproduce the compiled mesh within 50 nm");
  }

  void finish() {
    int nv = vertices.size(), nf = faces.size();
    if (nv < 4 || nf < 4) throw std::runtime_error("closed source mesh required");
    vn.assign(nv, {0, 0, 0}); fn.resize(nf);
    double volume6 = 0;
    for (int i = 0; i < nf; ++i) {
      for (int k = 0; k < 3; ++k) if (faces[i][k] < 0 || faces[i][k] >= nv) throw std::runtime_error("invalid source face");
      auto f = faces[i]; V a = vertices[f[0]], b = vertices[f[1]], c = vertices[f[2]];
      fn[i] = unit(cross(sub(b, a), sub(c, a))); volume6 += dot(a, cross(b, c));
      for (int k = 0; k < 3; ++k) {
        int vi = f[k], vj = f[(k+1)%3], vk = f[(k+2)%3];
        V u = sub(vertices[vj], vertices[vi]), v = sub(vertices[vk], vertices[vi]);
        double angle = std::atan2(norm(cross(u, v)), dot(u, v));
        vn[vi] = add(vn[vi], mul(fn[i], angle));
        auto key = std::minmax(vi, vj); auto& edge = edges[{key.first, key.second}];
        edge.normal = add(edge.normal, fn[i]); edge.count++; edge.balance += (vi < vj ? 1 : -1);
      }
    }
    if (!(volume6 > 0)) throw std::runtime_error("source must have positive outward-oriented volume");
    for (auto& edge : edges) {
      if (edge.second.count != 2 || edge.second.balance != 0) throw std::runtime_error("source edge is not closed consistently oriented two-manifold");
      edge.second.normal = unit(edge.second.normal);
    }
    for (auto& n : vn) n = unit(n);
    order.resize(nf); std::iota(order.begin(), order.end(), 0); nodes.reserve(2*nf); build(0, nf);
  }

  int build(int begin, int end) {
    Node n; n.begin = begin; n.end = end;
    n.lo = {INFINITY, INFINITY, INFINITY}; n.hi = {-INFINITY, -INFINITY, -INFINITY};
    for (int i = begin; i < end; ++i) for (int vi : faces[order[i]]) for (int k = 0; k < 3; ++k) {
      n.lo[k] = std::min(n.lo[k], vertices[vi][k]); n.hi[k] = std::max(n.hi[k], vertices[vi][k]);
    }
    int id = nodes.size(); nodes.push_back(n);
    if (end-begin <= 8) return id;
    int axis = 0; for (int k = 1; k < 3; ++k) if (n.hi[k]-n.lo[k] > n.hi[axis]-n.lo[axis]) axis = k;
    int middle = (begin+end)/2;
    std::nth_element(order.begin()+begin, order.begin()+middle, order.begin()+end, [&](int a, int b) {
      double ca = 0, cb = 0; for (int k = 0; k < 3; ++k) { ca += vertices[faces[a][k]][axis]; cb += vertices[faces[b][k]][axis]; } return ca < cb; });
    nodes[id].left = build(begin, middle); nodes[id].right = build(middle, end); return id;
  }

  double bound(V p, const Node& n) const {
    double s = 0; for (int k = 0; k < 3; ++k) { double d = std::max({n.lo[k]-p[k], 0., p[k]-n.hi[k]}); s += d*d; } return s;
  }
  void nearest(int ni, V p, double& best, int& face, Closest& hit) const {
    const auto& n = nodes[ni]; if (bound(p, n) > best) return;
    if (n.left < 0) {
      for (int i = n.begin; i < n.end; ++i) {
        int fi = order[i]; auto f = faces[fi];
        auto q = closest(p, vertices[f[0]], vertices[f[1]], vertices[f[2]]);
        double ds = dot(sub(p, q.p), sub(p, q.p)); if (ds < best) { best = ds; face = fi; hit = q; }
      }
      return;
    }
    int a = n.left, b = n.right; if (bound(p, nodes[b]) < bound(p, nodes[a])) std::swap(a, b);
    nearest(a, p, best, face, hit); nearest(b, p, best, face, hit);
  }

  double query(const mjtNum* ptr, mjtNum* gradient) const {
    V p = {ptr[0], ptr[1], ptr[2]}; double best = INFINITY; int fi = -1; Closest hit;
    nearest(0, p, best, fi, hit); if (fi < 0) mju_error("exact SDF BVH failed");
    auto f = faces[fi]; V pseudo = fn[fi]; std::vector<int> active;
    for (int k = 0; k < 3; ++k) if (hit.bary[k] > 1e-12) active.push_back(k);
    if (active.size() == 1) pseudo = vn[f[active[0]]];
    else if (active.size() == 2) { auto edge = std::minmax(f[active[0]], f[active[1]]); pseudo = edges.at({edge.first, edge.second}).normal; }
    V delta = sub(p, hit.p); double dist = std::sqrt(best), sign = dot(delta, pseudo) < 0 ? -1 : 1;
    V grad = dist > 1e-14 ? mul(delta, sign/dist) : pseudo;
    if (gradient) for (int k = 0; k < 3; ++k) gradient[k] = grad[k];
    return sign*dist;
  }
};

const char* kAttributes[] = {"file", "starts"};
int g_slot = -1;
mjfCollision g_stock_mesh_sdf = nullptr;
// Diagnostic counters for audits (collider calls, faces past the cull, faces
// that needed Frank-Wolfe, penetrating candidates). Not thread-safe; audits
// run the collider single-threaded.
long long g_stats[4] = {0, 0, 0, 0};

// Mesh-versus-exact-SDF narrowphase, installed only on request. MuJoCo 3.14's
// mjc_MeshSDF keeps every penetrating Frank-Wolfe start in a 50-slot buffer
// filled in BVH order and stops looking once it is full, so a few early faces
// can hide the deepest one. Here every mesh face is examined: an exact
// Lipschitz cull (the field changes by at most 1 m per m), its three corners
// evaluated exactly, Frank-Wolfe from the same Halton starts, and only the
// deepest point per face kept. Contacts are then chosen deepest first and by
// farthest-point sampling, built with MuJoCo's own mesh/SDF convention
// (normal into the SDF, position halfway along it). Other SDFs are passed to
// the stock collider unchanged.
int exact_mesh_collide(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2, mjtNum margin) {
  int instance = m->geom_plugin[g2];
  if (instance < 0 || m->plugin[instance] != g_slot || m->geom_type[g1] != mjGEOM_MESH)
    return g_stock_mesh_sdf(m, d, con, g1, g2, margin);
  const Mesh* field = reinterpret_cast<const Mesh*>(d->plugin_data[instance]);
  g_stats[0]++;
  const mjtNum *p1 = d->geom_xpos+3*g1, *R1 = d->geom_xmat+9*g1, *p2 = d->geom_xpos+3*g2, *R2 = d->geom_xmat+9*g2;
  // Mesh geom frame -> SDF geom frame: x2 = R2^T (R1 x1 + p1 - p2).
  double A[9], b[3], dp[3] = {p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]};
  for (int i = 0; i < 3; ++i) {
    b[i] = R2[i]*dp[0]+R2[3+i]*dp[1]+R2[6+i]*dp[2];
    for (int j = 0; j < 3; ++j) A[3*i+j] = R2[i]*R1[j]+R2[3+i]*R1[3+j]+R2[6+i]*R1[6+j];
  }
  int mid = m->geom_dataid[g1], va = m->mesh_vertadr[mid], nv = m->mesh_vertnum[mid];
  int fa = m->mesh_faceadr[mid], nf = m->mesh_facenum[mid];
  std::vector<V> local(nv);
  for (int i = 0; i < nv; ++i) {
    const float* v = m->mesh_vert+3*(va+i);
    for (int k = 0; k < 3; ++k) local[i][k] = A[3*k]*v[0]+A[3*k+1]*v[1]+A[3*k+2]*v[2]+b[k];
  }
  const Node& box = field->nodes[0];
  int starts = field->starts ? field->starts : std::max(1, m->opt.sdf_initpoints), iterations = m->opt.sdf_iterations;
  std::vector<V> points; std::vector<double> depth;
  for (int f = 0; f < nf; ++f) {
    const int* face = m->mesh_face+3*(fa+f);
    V c[3] = {local[face[0]], local[face[1]], local[face[2]]};
    bool apart = false;
    for (int k = 0; k < 3; ++k)
      apart |= std::max({c[0][k], c[1][k], c[2][k]}) < box.lo[k] || std::min({c[0][k], c[1][k], c[2][k]}) > box.hi[k];
    if (apart) continue;
    V centroid = mul(add(add(c[0], c[1]), c[2]), 1./3);
    double radius = std::max({norm(sub(c[0], centroid)), norm(sub(c[1], centroid)), norm(sub(c[2], centroid))});
    double at_centroid = field->query(centroid.data(), nullptr);
    if (at_centroid >= radius) continue;
    g_stats[1]++;
    // Lower bound on the field over the face from each sample p: f(p) - max |x - p|.
    double lower = at_centroid-radius, corner[3], edge[3] = {norm(sub(c[1], c[0])), norm(sub(c[2], c[1])), norm(sub(c[0], c[2]))};
    V best_x = c[0]; double best = INFINITY;
    for (int k = 0; k < 3; ++k) {
      corner[k] = field->query(c[k].data(), nullptr);
      lower = std::max(lower, corner[k]-std::max(edge[k], edge[(k+2)%3]));
      if (corner[k] < best) { best = corner[k]; best_x = c[k]; }
    }
    // Frank-Wolfe can only matter if some interior point might lie below both
    // zero and the best corner; otherwise the corners are provably enough.
    if (lower < std::min(best, 0.)) g_stats[2]++;
    if (lower < std::min(best, 0.))
    for (int s = 0; s < starts; ++s) {
      double u = mju_Halton(s+1, 2), w = mju_Halton(s+1, 3);
      if (u+w > 1) { u = 1-u; w = 1-w; }
      V x = add(add(mul(c[0], 1-u-w), mul(c[1], u)), mul(c[2], w));
      for (int step = 0; step < iterations; ++step) {
        mjtNum g[3]; field->query(x.data(), g);
        int arg = 0; double low = INFINITY;
        for (int k = 0; k < 3; ++k) { double v = c[k][0]*g[0]+c[k][1]*g[1]+c[k][2]*g[2]; if (v < low) { low = v; arg = k; } }
        x = add(x, mul(sub(c[arg], x), 2./(step+2.)));
      }
      double v = field->query(x.data(), nullptr);
      if (v < best) { best = v; best_x = x; }
    }
    if (best < 0) { points.push_back(best_x); depth.push_back(best); g_stats[3]++; }
  }
  // Deepest first, then farthest-point sampling; coincident points (shared
  // corners of neighbouring faces) are never selected twice.
  int limit = std::min(std::max(1, m->opt.sdf_initpoints), mjMAXCONPAIR), n = points.size(), cnt = 0;
  std::vector<double> gap(n, INFINITY);  // squared distance to the nearest chosen point; -1 once chosen
  int pick = n ? int(std::min_element(depth.begin(), depth.end())-depth.begin()) : -1;
  while (pick >= 0 && cnt < limit) {
    gap[pick] = -1;
    mjtNum g[3]; field->query(points[pick].data(), g);
    V normal = {-(R2[0]*g[0]+R2[1]*g[1]+R2[2]*g[2]), -(R2[3]*g[0]+R2[4]*g[1]+R2[5]*g[2]), -(R2[6]*g[0]+R2[7]*g[1]+R2[8]*g[2])};
    double length = norm(normal);
    if (length > mjMINVAL) {
      normal = mul(normal, 1/length);
      const V& x = points[pick];
      mjPreContact& out = con[cnt++];
      out.dist = depth[pick];
      for (int k = 0; k < 3; ++k) {
        out.normal[k] = normal[k]; out.tangent[k] = 0;
        out.pos[k] = R2[3*k]*x[0]+R2[3*k+1]*x[1]+R2[3*k+2]*x[2]+p2[k]+.5*depth[pick]*normal[k];
      }
    }
    const V chosen = points[pick];
    pick = -1; double far = mjMINVAL*mjMINVAL;
    for (int i = 0; i < n; ++i) if (gap[i] >= 0) {
      V e = sub(points[i], chosen); gap[i] = std::min(gap[i], dot(e, e));
      if (gap[i] > far) { far = gap[i]; pick = i; }
    }
  }
  return cnt;
}

void register_plugin() {
  mjpPlugin p; mjp_defaultPlugin(&p);
  p.name = "g4.exact_mesh_sdf.v2"; p.capabilityflags = mjPLUGIN_SDF;
  p.nattribute = 2; p.attributes = kAttributes;
  p.nstate = +[](const mjModel*, int) { return 0; };
  p.init = +[](const mjModel* m, mjData* d, int id) {
    try { d->plugin_data[id] = reinterpret_cast<uintptr_t>(new Mesh(m, id)); return 0; }
    catch (const std::exception& e) { mju_warning("exact mesh SDF init failed: %s", e.what()); return -1; }
  };
  p.destroy = +[](mjData* d, int id) { delete reinterpret_cast<Mesh*>(d->plugin_data[id]); d->plugin_data[id] = 0; };
  p.copy = +[](mjData* dest, const mjModel*, const mjData* src, int id) {
    *reinterpret_cast<Mesh*>(dest->plugin_data[id]) = *reinterpret_cast<const Mesh*>(src->plugin_data[id]); };
  p.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  p.compute = +[](const mjModel*, mjData*, int, int) {};
  p.sdf_distance = +[](const mjtNum x[3], const mjData* d, int id) { return reinterpret_cast<const Mesh*>(d->plugin_data[id])->query(x, nullptr); };
  p.sdf_gradient = +[](mjtNum out[3], const mjtNum x[3], const mjData* d, int id) { reinterpret_cast<const Mesh*>(d->plugin_data[id])->query(x, out); };
  p.sdf_staticdistance = +[](const mjtNum[3], const mjtNum*) -> mjtNum { mju_error("exact mesh SDF requires explicit original mesh; no marching cubes"); return 0; };
  p.sdf_attribute = +[](mjtNum*, const char**, const char**) {};
  p.sdf_aabb = +[](mjtNum[6], const mjtNum*) { mju_error("exact mesh SDF requires explicit original mesh bounds"); };
  g_slot = mjp_registerPlugin(&p);
}
}  // namespace

// Diagnostics for audits: which geometry mode an instance uses and its
// measured deviation from the compiled float32 triangles.
extern "C" {
const char* g4_exact_mesh_sdf_mode(const mjData* d, int instance) {
  return reinterpret_cast<const Mesh*>(d->plugin_data[instance])->mode.c_str();
}
double g4_exact_mesh_sdf_compiled_deviation(const mjData* d, int instance) {
  return reinterpret_cast<const Mesh*>(d->plugin_data[instance])->compiled_deviation;
}
int g4_exact_mesh_sdf_counts(const mjData* d, int instance, int* vertices, int* faces) {
  const Mesh* mesh = reinterpret_cast<const Mesh*>(d->plugin_data[instance]);
  *vertices = mesh->vertices.size(); *faces = mesh->faces.size(); return 0;
}
// Copy the diagnostic counters into out[4] and optionally reset them.
void g4_exact_mesh_sdf_stats(long long* out, int reset) {
  for (int k = 0; k < 4; ++k) { out[k] = g_stats[k]; if (reset) g_stats[k] = 0; }
}
// Install (1) or restore (0) the stock mesh/SDF narrowphase; returns whether
// the exact collider is now active. Process-wide, like mjCOLLISIONFUNC.
int g4_exact_mesh_sdf_set_collider(int enable) {
  if (!g_stock_mesh_sdf) g_stock_mesh_sdf = mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_SDF];
  mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_SDF] = enable ? exact_mesh_collide : g_stock_mesh_sdf;
  return mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_SDF] == exact_mesh_collide;
}
}

mjPLUGIN_LIB_INIT(g4_exact_mesh_sdf) { register_plugin(); }
