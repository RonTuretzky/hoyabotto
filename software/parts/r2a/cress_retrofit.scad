// Cress retrofit R2a. Dimensions in millimetres, original XY and Z datum.
// Derivative of Macce's Self-watering Cress or Microgreens Planter.
// https://www.printables.com/model/434505
// CC BY-SA 4.0, attribution recorded in design.json.
// Prototype only. No machine motion or print profile is included.
part = "carrier";
$fn = 48;

module old_holder() { import("source/cressmaster-holder.stl", convexity=12); }
module trough() { import("source/cressmaster-trough.stl", convexity=12); }
module old_inset(x) {
    translate([x+45,0,0]) import("source/cressmaster-inset.stl", convexity=12);
}
module capsule(w,l,h) {
    linear_extrude(h) hull() for(y=[-(l-w)/2,(l-w)/2]) translate([0,y]) circle(d=w);
}
module carrier() {
    union() {
        old_holder();
        for(x=[-45,-15,15,45]) {
            old_inset(x);
            // Bond each inset to the plate with a continuous collar.
            // Opening remains wider than the original inset cavity.
            translate([x,0,-1.5]) difference() {
                capsule(11,70,1.5);
                translate([0,0,-.1]) capsule(6.3,65.4,1.7);
            }
        }
        for(y=[-36.5,36.5]) {
            // 24 mm long, 6 mm thick jaws contact above the rim.
            translate([-12,y-3,-1.5]) cube([24,6,24.5]);
        }
    }
}
module retainer() {
    // Intended seated Z = 1.0 relative to original grow deck top.
    // 1 mm is a starting material-stack allowance, not a measured paper fit.
    union() {
        difference() {
            translate([-74,-33,0]) cube([148,66,2.4]);
            translate([-70,-29,-.1]) cube([140,58,2.6]);
        }
        for(x=[-72,72]) translate([x-2,-10,0]) cube([4,20,18]);
    }
}
module coupon() {
    // Dry grip test. Root and fin cross section match carrier features.
    translate([-18,-12,0]) cube([36,24,3]);
    translate([-12,-3,2]) cube([24,6,23]);
}
// Print exports have Z=0 minimum. Original source files remain unmodified.
if(part=="carrier") translate([0,0,20.5]) carrier();
if(part=="retainer") retainer();
if(part=="grip_coupon") coupon();
if(part=="assembly") { trough(); carrier(); translate([0,0,1]) retainer(); }
if(part=="carrier_collision") intersection() { carrier(); trough(); }
if(part=="baseline_collision") intersection() { old_holder(); trough(); }
if(part=="retainer_collision") intersection() { carrier(); translate([0,0,1]) retainer(); }
