/**
* Name: placedesterreaux
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model graphLyon
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../Continuum_Crowds/high_density_zone.gaml'
import '../../env.gaml'
import '../../helpers/detection.gaml'
/* Insert your model definition here */

global {
	float step <- STEP;
	
	file building_shape_file <- shape_file('../../../includes/place_des_Terreaux/ok.shp');
	file road_shape_file <- shape_file('../../../includes/place_des_Terreaux/lines.shp');
	file center_shape_file <- shape_file('../../../includes/place_des_Terreaux/center.shp');
	file init_space_shape_file <- shape_file('../../../includes/place_des_Terreaux/init_space.shp');
	
	geometry shape <- envelope(building_shape_file);
	geometry space <- copy(shape);
	point group_target <- {30, 30};
	
	
	init {
		create building from: building_shape_file;
		create center from: center_shape_file;
		create init_space from: init_space_shape_file;
		
		create road from: road_shape_file {
			shape <- shape + 0.3;
		}
		
		graph_network <- as_edge_graph(road_shape_file);
//		write graph_network;

		create high_density_zone {
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 15;
			boundary <- [50.0, 70, 50, 70];
			CELL_SIZE_X <- (boundary[1] - boundary[0]) / GRID_WIDTH;
		    CELL_SIZE_Y <- (boundary[3] - boundary[2]) / GRID_HEIGHT;
		 	CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			nb_groups <- 1;
			
			float x_min <- boundary[0];
			float y_min <- boundary[2];
			group_goals <- [1:: [[1, 0]]];
			do init_state;
		}
		
		// operational level
		create avoiding number: 1;
//		create walking_directly number: 1;
		create following number: 1;
		
		// tatical level
		create best_adjacient_node number: 1;

	}
	
	reflex init_people when: every(3#cycle) {
		create individual number: 1 {
			color <- #blue;
			location <- any_location_in(one_of(init_space));
//			target <- any_location_in(first(center));
			target <- {30, 35};
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
//			my_operational_behavior <- first(walking_directly);
			my_operational_behavior <- first(avoiding);
			my_tactical_behavior <- first(best_adjacient_node);
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
//		list<float> bound <- detect_dense_region(my_data, 1.0, 10, 300, 2.0);
//		
//		if length(bound) > 0 {
//			float x_min <- bound[0];
//			float x_max <- bound[1];
//			float y_min <- bound[2];
//			float y_max <- bound[3];
//			
////			ask first(high_density_zone) {
////				shape <- polyline([{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}, {x_min, y_min}]);
////			}
//		}
//		
//	}
	
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

experiment exp parallel: true {
	output {
		display my_display {
			species building;
//			species center;
//			species init_space;
			species high_density_zone;
			species road;
			species individual;
		}
	}
}