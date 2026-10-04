// R3.3: nested basket, heavier gravity ring, raised retainer grip, mm. Not physically tested.
// Upright print orientation. The basket front is OPEN for watering cloth.
// Direct robot pickup from the printer plate is assumed; no plate hardware.
part="basket";
$fn=48; eps=.02;
basket_z=27; cloth_t=2; grow_pad_t=3;
retainer_z=basket_z+2.4+cloth_t+grow_pad_t; // 34.4 mm
module rect2(w,d,c=4){polygon([[-w/2+c,-d/2],[w/2-c,-d/2],[w/2,-d/2+c],[w/2,d/2-c],[w/2-c,d/2],[-w/2+c,d/2],[-w/2,d/2-c],[-w/2,-d/2+c]]);}
module slab(w,d,h,c=4){linear_extrude(h)rect2(w,d,c);}
module taper(w0,d0,w1,d1,h){hull(){slab(w0,d0,eps);translate([0,0,h-eps])linear_extrude(eps)offset(r=(w1-w0)/2)rect2(w0,d0);}}
module reservoir(){
 union(){
  difference(){slab(140,94,45);translate([0,0,2])slab(134,88,45,3);}
  for(s=[-1,1])translate([s*72.9-3.1,-12,0])cube([6.2,24,16]);
  // Permanent integral basket rests. These are NOT removable print supports.
  for(x=[-35,0,35])translate([x-2,-16,1.9])cube([4,32,basket_z-1.9]);
 }
}
module basket_body(){
 union(){
  difference(){
   taper(96,50,132,86,18);
   translate([0,0,2.4])taper(91.2,45.2,126.4,80.4,17.6);
   for(y=[-18,-12,-6,0,6,12,18])translate([-40,y-1.2,-.1])cube([80,2.4,2.6]);
  }
  for(s=[-1,1])scale([s,1,1]){
   hull(){translate([64,-12,17.9])cube([1.8,24,.1]);translate([60,-12,22])cube([6,24,.1]);}
   translate([60,-12,22])cube([6,24,16]);
  }
 }
}
module basket(){difference(){
 basket_body();
 // 92 mm clear width for 88 mm cloth, 2 mm clearance on each side.
 // Open to the top; cut reaches the edge of the mat platform at Y=-21.
 translate([-46,-60,-.1])cube([92,39,60]);
}}
module retainer_base(){polygon([[-41.6,-22.6],[41.6,-22.6],[45.6,-18.6],[45.6,18.6],[41.6,22.6],[-41.6,22.6],[-45.6,18.6],[-45.6,-18.6]]);}
module retainer_profile(r){offset(r=r)retainer_base();}
module retainer_taper(r0,r1,h){hull(){linear_extrude(eps)retainer_profile(r0);translate([0,0,h-eps])linear_extrude(eps)retainer_profile(r1);}}
module retainer(){union(){
 difference(){retainer_taper(4.7,13.7,9);translate([0,0,-.1])retainer_taper(.6,9.8,9.2);}
 for(s=[-1,1])scale([s,1,1])translate([42,-4,0])cube([8.2,8,.8]);
 for(s=[-1,1])scale([1,s,1])translate([-4,19,0])cube([8,8.2,.8]);
 // Dedicated rear grip. Outside the grow pad (Y <= 21), integral to the rim.
 // 45-degree outward flare avoids a suspended overhang during upright printing.
 hull(){translate([-8,25.8,0])cube([16,1.5,.02]);translate([-8,25.8,4.5])cube([16,6,.02]);}
 translate([-8,25.8,4.5])cube([16,6,29.5]);
 // Pinch the top 10 mm across Y. At the 34.4 mm seat, jaw tips stay at
 // Z >= 58.4, above the 45 mm rear rim. Fully seat, then open and lift away.
}}
module assembly(){reservoir();translate([0,0,basket_z])basket();translate([0,0,retainer_z])retainer();}
module plate(){translate([110,161,0])reservoir();translate([110,48,0])retainer();}
if(part=="reservoir")reservoir();
if(part=="basket")basket();
if(part=="retainer")retainer();
if(part=="assembly")assembly();
if(part=="plate")plate();
