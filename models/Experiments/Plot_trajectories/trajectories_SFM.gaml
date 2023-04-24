/**
* Name: trajectoriesSFM
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model trajectoriesSFM
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../env.gaml'
import '../../helpers/detection.gaml'

/* Insert your model definition here */

global {
	float step <- STEP;
	
	file obstacle_shape_file <- shape_file('obstacle.shp');
	file node_shape_file <- shape_file('node.shp');
	file mesh_shape_file <- shape_file('mesh.shp');
	
//	geometry shape <- envelope(mesh_shape_file);
	
	float room_width <- 60.0 #m;
	float room_length <- 40.0 #m;
	float door_width <- 3.0 #m;
		
	init {
		create nav_mesh from: mesh_shape_file;
		create mesh_centroid from: node_shape_file;
		create building from: obstacle_shape_file;
		
		loop c over: mesh_centroid {
			loop my_mesh over: nav_mesh {
				if c.shape overlaps	my_mesh.shape {
					my_mesh.centroid <- c.shape.location;
				}
			}	
		}
		
		loop mesh_i over: nav_mesh {
			loop mesh_j over: nav_mesh {
				if (mesh_j != mesh_i) {
					if mesh_i.shape intersects mesh_j.shape {
						mesh_i.neighbors <- mesh_i.neighbors + [mesh_j];
					}
				}
			}
		}
		
//		create walking_directly number: 1;
		create avoiding_SFM number: 1;
		create best_neighbor_mesh number: 1;
		
		create room {			
			shape <- polyline([{0, 0}, {room_width, 0}, {room_width, room_length}, {0, room_length}, {0, 0}]) 
					 - polyline([{room_width,  (room_length - door_width) / 2}, {room_width,  (room_length + door_width) / 2}]);
		}
		
		create individual number: 1 {
			color <- #blue;
			target <- {room_width, room_length / 2 + 0.5};
			local_target <- target;
			
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
			my_operational_behavior <- first(avoiding_SFM);
//			my_tactical_behavior <- first(best_neighbor_mesh);
			
//			current_mesh <- one_of([nav_mesh(6)]);
//			location <- any_location_in(current_mesh);
			
			location <- {1.6771860484013772,27.60428474002328,0.0};
//			my_zone <- first(high_density_zone);
		}
		
	}
	
	// every(3#cycle)
	reflex init_people when: false {
		create individual number: 1 {
			color <- #blue;
			target <- {room_width, room_length / 2 + 1.0};
			local_target <- target;
			
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
			my_operational_behavior <- first(avoiding);
//			my_tactical_behavior <- first(best_neighbor_mesh);
			
//			current_mesh <- one_of([nav_mesh(6)]);
			location <- any_location_in(current_mesh);
			
		}
	}
	
}

species mesh_centroid {
	aspect default {
		draw circle(0.1) at: shape.location color: #red border: #green;
	}
}

species room {
	aspect default {
		draw (shape + 0.2) color: #grey;
	}
}


experiment exp parallel: true {
	output {
		display my_display {
			species room;
			species building;
//			species nav_mesh;
//			species mesh_centroid;
			species individual;
		}
	}
}