/**
* Name: meshLyon
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model meshLyon
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../env.gaml'
import '../../helpers/detection.gaml'
/* Insert your model definition here */

global {
	float step <- STEP;
	
	file building_shape_file <- shape_file('../../../includes/place_des_Terreaux/ok.shp');
	file road_shape_file <- shape_file('../../../includes/place_des_Terreaux/lines.shp');
	file center_shape_file <- shape_file('../../../includes/place_des_Terreaux/center.shp');
	file init_space_shape_file <- shape_file('../../../includes/place_des_Terreaux/init_space.shp');
	
	file nav_mesh_shape_file <- shape_file('../../../includes/gen_map_official/mesh.shp');
	file mesh_centroid_shape_file <- shape_file('../../../includes/gen_map_official/node.shp');
	
	geometry shape <- envelope(building_shape_file);
	geometry space <- copy(shape);
	point group_target <- {60, 60};
	
	init {
		
		create building from: building_shape_file;
//		create center from: center_shape_file;
//		create init_space from: init_space_shape_file;
		
		create nav_mesh from: nav_mesh_shape_file;
		create mesh_centroid from: mesh_centroid_shape_file;
		
//		loop my_mesh over: nav_mesh {
//			list<point> points <- first(my_mesh.shape.points) = last(my_mesh.shape.points) ? my_mesh.shape.points - last(my_mesh.shape.points): my_mesh.shape.points;
//			point c;
//			loop p over: points {
//				c <- c + p;
//			}
//			ask my_mesh {
//				centroid <- c / length(points);
//			}
//		}

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

		// operational level
		create walking_directly number: 1;
		create avoiding number: 1;
		create following number: 1;
		
		// tatical level
//		create best_adjacient_node number: 1;
		create best_neighbor_mesh number: 1;
		
		create high_density_zone {
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 15;
			boundary <- [50.0, 80, 50, 70];
			CELL_SIZE_X <- (boundary[1] - boundary[0]) / GRID_WIDTH;
		    CELL_SIZE_Y <- (boundary[3] - boundary[2]) / GRID_HEIGHT;
		 	CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			nb_groups <- 1;

			group_goals <- [1::[[0, 5]]];
			exit_meshes <- [1::nav_mesh(75)];
			do init_state;
		}
		
		create ok_target {
			shape <- {30, 35};
			shape <- shape + 1.0;
		}

	}
	
	reflex init_people when: every(3#cycle) {
		create individual number: 1 {
			color <- #blue;
			target <- {30, 35};
			
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
//			my_operational_behavior <- first(walking_directly);
			my_operational_behavior <- first(avoiding);
			
//			my_tactical_behavior <- first(best_adjacient_node);
			my_tactical_behavior <- first(best_neighbor_mesh);
			
			current_mesh <- one_of([nav_mesh(93), nav_mesh(102)]);
//			current_mesh <- one_of([nav_mesh(93), nav_mesh(102), nav_mesh(40)]);
			location <- any_location_in(current_mesh);
			my_zone <- first(high_density_zone);
		}
	}
	
	//when: cycle > 10 and every(100#cycle)
//	reflex detect_high_density_zone when: cycle > 10 and every(20#cycle) {
//		list<list<float>> my_data;
//		loop ag over: individual {
//			list<float> ag_coordinate <- [ag.location.x, ag.location.y];
//			my_data <- my_data + [ag_coordinate];
//		}
//
//		list<float> bound <- detect_dense_region(my_data, 1.0, 10, 100, 2.0);
//		
//		if length(bound) > 0 {
//			float x_min <- bound[0];
//			float x_max <- bound[1];
//			float y_min <- bound[2];
//			float y_max <- bound[3];
//			
//			ask first(high_density_zone) {
//				shape <- polyline([{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}, {x_min, y_min}]);
//			}
//		}
//		
//	}
	
	reflex write_status when: every(100#cycle) {
		write length(individual);
	}
	
}


species center {
	aspect default {
		draw shape color: #yellow;
	}
}

species init_space {
	aspect default {
		draw shape color: #yellow;
	}
}

species mesh_centroid {
	aspect default {
		draw circle(0.1) at: shape.location color: #red border: #green;
	}
}


species ok_target {
	aspect default {
		draw shape color: #green;
	}
}

experiment exp parallel: true {
	output {
		display my_display {
			species building;
//			species center;
//			species init_space;
			species nav_mesh;
//			species mesh_centroid;
			species high_density_zone;
			species ok_target;
			species road;
			species individual;
		}
	}
}