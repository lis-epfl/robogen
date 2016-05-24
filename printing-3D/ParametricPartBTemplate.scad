// title      : RoboGen parametric B part
// author     : Alice Concordel

partB(20); //the parameter is the angle of the parametric part

module partB(alpha)
{
//plate parameters
	lengthSlide = 34;
	widthSlide = 34;
	heightSlide = 1.5;
	
	radiusHole = 1;

//wire hole parameters
	lengthWire = 3;
	widthWire = 9;
	heightWire = heightSlide;

//joint base parameters
	lengthBase = 10;
	widthBase = 20;
	heightBase = 6;
	
//cylinder parameters
	radiusCylinderBase = 5;
	heightCylinderBase = 20;
	
//rotating cube top part parameters
	lengthTop = 9;
	widthTop = 20;
	heightTop = 5;
	
	lengthEnd = 4;
	widthEnd = 15;
	heightEnd = 10;
	
	
	union()
	{
		difference()
		{
			// starting plate
			cube([lengthSlide,widthSlide,1.5]);
			
			union()
			{
			// chamfer the edges of the plate
				rotate ([0,-45,0]) cube([3,widthSlide,3]);
				rotate([45,0,0]) cube([lengthSlide,3,3]);
				translate([lengthSlide,0,0]) rotate([0,-45,0]) cube([3,widthSlide,3]);
				translate([0,widthSlide,0]) rotate([45,0,0]) cube([lengthSlide,3,3]);

			// holes in the plate
					//round holes
				translate([4.5,4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				translate([lengthSlide-4.5,4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				translate([4.5,widthSlide-4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				translate([lengthSlide-4.5,widthSlide-4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				
					//rectangular wire holes
				translate([7-lengthWire/2,widthSlide/2-widthWire/2,-0.5]) cube([lengthWire,widthWire,heightWire+1]);
				translate([lengthSlide-7-lengthWire/2,widthSlide/2-widthWire/2,-0.5]) cube([lengthWire,widthWire,heightWire+1]);
			}
		}
		// base of shaft part
		translate([lengthSlide/2-10/2,widthSlide/2-20/2,0]) cube([lengthBase,widthBase,heightBase+heightSlide]);
	
		//chamfer the edges of the shaft base
		difference()
		{
			translate([lengthSlide/2-(lengthBase+3)/2,widthSlide/2-(widthBase+3)/2]) cube([lengthBase+3,widthBase+3,3]);
			union()
			{
			translate([lengthSlide/2-lengthBase/2-heightSlide,0,heightSlide]) rotate ([0,-45,0]) cube([heightSlide*sqrt(2),34,heightSlide*sqrt(2)]);
			translate([0,widthSlide/2-widthBase/2-heightSlide,heightSlide]) rotate([45,0,0]) cube([lengthSlide,heightSlide*sqrt(2),heightSlide*sqrt(2)]);
			translate([lengthSlide/2+lengthBase/2+heightSlide,0,heightSlide]) rotate([0,-45,0]) cube([heightSlide*sqrt(2),34,heightSlide*sqrt(2)]);
			translate([0,widthSlide/2+widthBase/2+heightSlide,heightSlide]) rotate([45,0,0]) cube([lengthSlide,heightSlide*sqrt(2),heightSlide*sqrt(2)]);
			}
		}

		// cylinder or rounded part
		translate([lengthSlide/2,widthSlide/2-heightCylinderBase/2,heightBase+heightSlide]) rotate([-90,0,0]) cylinder(r = radiusCylinderBase, h = heightCylinderBase, $fn = 30);
		
		// next part of join = Box over one smaller Box
		//========================== angle parameter below
		translate([lengthSlide/2,widthSlide/2-widthTop/2,heightSlide+heightBase]) rotate([0,alpha,0]) translate([-lengthTop/2,0,0]) cube([lengthTop,widthTop,heightTop]);
		translate([lengthSlide/2,widthSlide/2-widthEnd/2,heightSlide+heightBase]) rotate([0,alpha,0]) translate([-lengthEnd/2,0,0]) cube([lengthEnd,widthEnd,heightEnd+heightTop]);
	}
}
