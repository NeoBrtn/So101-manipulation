// Bottle fixture for phase 2 (decapping): holds a petaloid-base bottle upright and
// stops it from turning while the arm unscrews the cap. No moving parts: the ribs on
// the floor slot into the grooves between the bottle's feet (form closure).
//
//   part = "fixture"  the full cup with its mounting flange
//   part = "coupon"   floor + ribs + a low ring only: a 15 min print to check the fit
//
// Export:  openscad -D 'part="coupon"' -o coupon.stl bottle_fixture.scad
// All sizes in mm. Measured bottle: Auchan "finement petillante" 50 cl, 220 mm tall,
// body diameter 60, 5 feet, grooves between feet ~3 mm wide at the table and rising
// to ~15 mm at the rim.

part = "fixture";

/* [Bottle] */
bottle_d    = 60;    // body diameter
feet        = 5;     // number of feet (= number of grooves = number of ribs)

/* [Fit] */
clearance   = 1.0;   // radial gap between the bottle body and the cup wall
lead_in     = 6;     // funnel: the cup opening is this much wider (radius) at the top
lead_in_h   = 8;     // funnel height

/* [Cup] */
cup_h       = 35;    // wall height above the floor (the feet zone is ~25 mm tall)
wall        = 3;
floor_t     = 3;

/* [Ribs] (wedges sitting in the grooves between the feet) */
rib_w       = 3.0;   // width at the base: narrower than the groove at the table
rib_r_in    = 14;    // inner end (the feet merge into the centre dome there)
rib_h_in    = 4;     // height at the inner end
rib_h_out   = 10;    // height at the wall (the groove is ~15 mm high at the rim)
rib_tip     = 0.8;   // width of the ridge (pointed: a misaligned bottle twists in)

/* [Mounting] */
flange_d    = 110;
flange_t    = 4;
hole_d      = 4.5;   // M4 screws, or use double-sided tape and ignore them
holes       = 3;

$fn = 96;

inner_r = bottle_d / 2 + clearance;
outer_r = inner_r + wall;

module rib() {
    // Wedge along +x: triangular cross-section, rising toward the wall
    hull() {
        translate([rib_r_in, 0, 0]) rib_section(rib_h_in);
        translate([inner_r - 0.01, 0, 0]) rib_section(rib_h_out);
    }
}

module rib_section(h) {
    rotate([90, 0, 90])
        linear_extrude(0.01)
            polygon([[-rib_w / 2, 0], [rib_w / 2, 0], [rib_tip / 2, h], [-rib_tip / 2, h]]);
}

module ribs() {
    for (i = [0 : feet - 1]) rotate([0, 0, i * 360 / feet]) translate([0, 0, floor_t]) rib();
}

module outer_with_funnel() {
    // thicken the wall where the funnel cuts into it
    cylinder(r = outer_r, h = floor_t + cup_h - lead_in_h);
    translate([0, 0, floor_t + cup_h - lead_in_h])
        cylinder(r1 = outer_r, r2 = inner_r + lead_in + 1.2, h = lead_in_h);
}

module fixture() {
    difference() {
        union() {
            outer_with_funnel();
            cylinder(d = flange_d, h = flange_t);
        }
        translate([0, 0, floor_t]) cylinder(r = inner_r, h = cup_h + 1);
        translate([0, 0, floor_t + cup_h - lead_in_h])
            cylinder(r1 = inner_r, r2 = inner_r + lead_in, h = lead_in_h + 0.01);
        for (i = [0 : holes - 1])
            rotate([0, 0, 90 + i * 360 / holes])
                translate([(flange_d / 2 + outer_r) / 2, 0, -1]) cylinder(d = hole_d, h = flange_t + 2);
    }
    ribs();
}

module coupon() {
    difference() {
        cylinder(r = outer_r, h = floor_t + 8);
        translate([0, 0, floor_t]) cylinder(r = inner_r, h = 10);
    }
    ribs();
}

if (part == "coupon") coupon(); else fixture();
