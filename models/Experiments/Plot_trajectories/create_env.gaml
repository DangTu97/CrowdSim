/**
* Name: createenv
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model createenv
import '../../helpers/generate_medial_axis_function.gaml'
/* Insert your model definition here */

global {
	float room_width <- 60.0 #m;
	float room_length <- 40.0 #m;
	float door_width <- 2.0 #m;
	
	float obstacle_width <- 3.5 #m;
	float obstacle_length <- 10.0 #m;
	
//	geometry shape <- envelope(room);

	init {
		create room {
//			shape <- polyline([{0, 0}, {room_width, 0}, {room_width, room_length}, {0, room_length}, {0, 0}]) 
//					 - polyline([{room_width,  (room_length - door_width) / 2}, {room_width,  (room_length + door_width) / 2}]);
		
//			shape <- polyline([{0, 0}, {room_width, 0}, {room_width, room_length}, {0, room_length}, {0, 0}]);

			shape <- polygon([{0, 0}, {room_width, 0}, {room_width, room_length}, {0, room_length}, {0, 0}]);
			
		}
		
		create obstacle {
			shape <- rectangle(obstacle_width, obstacle_length) at_location {room_width / 4, room_length / 2};
		}
		
		create obstacle {
			shape <- rectangle(obstacle_width, obstacle_length) at_location {room_width / 2, room_length * 1 / 4};
		}
		
		create obstacle {
			shape <- rectangle(obstacle_width, obstacle_length) at_location {room_width / 2, room_length * 3 / 4};
		}
		
		create obstacle {
			//shape <- rectangle(obstacle_width, obstacle_length) at_location {room_width * 3 / 4, room_length / 2};
			
			geometry geom0 <- rectangle(obstacle_width, obstacle_length) at_location {room_width * 3 / 4, room_length / 2};
			geometry geom1 <- rectangle(obstacle_width*0.85, obstacle_width/2) at_location {room_width * 3/4 - obstacle_width/2, room_length/2 - obstacle_length/2 + obstacle_width/4};
			geometry geom2 <- rectangle(obstacle_width*0.85, obstacle_width/2) at_location {room_width * 3/4 - obstacle_width/2, room_length/2 + obstacle_length/2 - obstacle_width/4};
			shape <- union(geom0, geom1, geom2);
//			write shape;
			
//			shape <- polygon([{42, 16.5}, {43.5, 16.5}, {43.5, 23.5}, {42, 23.5}, {42, 25}, {43.5, 25}, {45, 25}, {46.5, 25}, {46.5, 15}, {45, 15}, {43.5, 15}, {42, 15}, {42, 16.5}]);
		}
		
		list<geometry> obstacles;		
		loop ob over: obstacle {
	    	obstacles <- obstacles + ob.shape;
	    }
	    	    
	   	geometry bound <- room(0).shape;	
		list<list> results <- generate_medial_axis(bound, obstacles, 8.0, 5.0);

		list<geometry> medial_axis <- results[0];
		loop ax over: medial_axis {
			create my_edge {
				shape <- ax;
			}
		}
//		write medial_axis;

		list<geometry> nav_meshes <- results[1];
		loop i from: 0 to: length(nav_meshes) - 1 {
			create my_nav_mesh {
				shape <- nav_meshes[i];
				list<point> points <- first(shape.points) = last(shape.points) ? shape.points - last(shape.points): shape.points;
				loop p over: points {
					centroid <- centroid + p;
				}
				centroid <- centroid / length(points);
				
				bool at_corner <- true;
				loop j from: 0 to: length(points) - 2 {
					if (polyline([points[j], points[j + 1]]) in medial_axis) or 
					(polyline([points[j+1], points[j]]) in medial_axis) {
						at_corner <- false;
						break;
					}
				}
				
				is_corner_mesh <- at_corner;
				my_color <- is_corner_mesh ? #blue : #white;
				
			}
		}
		
		loop nav over: my_nav_mesh {
			create nav_centroid {
				center <- nav.centroid;
				shape <- center;
			}	
		}
		
		save my_nav_mesh type: shp to: "mesh.shp" attributes: ["is_corner"::is_corner_mesh];
		save nav_centroid type: shp to: "node.shp";
		save obstacle type: shp to: "obstacle.shp";
		
	}
}

species room {
	aspect default {
		draw (shape + 0.2) color: #grey;
	}
}

species obstacle {
	aspect default {
		draw shape color: #white border: #black;
	}
}

species my_edge {
	aspect default {
		draw shape color: #red;
	}
}

species nav_centroid {
	point center;
	aspect default {
		draw circle(0.1) color:#red at: center;
	}
}

species my_nav_mesh {
	point centroid;
	bool is_corner_mesh;
	rgb my_color;
	aspect default {
		draw shape color: my_color border: #green;
//		draw circle(0.1) color:#red at: centroid;
	}
}

experiment exp parallel: true {
	output {
		display my_display {
			species room;
			species obstacle;
			species my_nav_mesh;
			species nav_centroid;
			species my_edge;
		}
	}
}