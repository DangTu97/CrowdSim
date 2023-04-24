/**
* Name: trajectoriesSFMplugin
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model trajectoriesSFMplugin
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../env.gaml'
import '../../helpers/detection.gaml'
/* Insert your model definition here */

global {
	float step <- 0.2;
	
	file obstacle_shape_file <- shape_file('obstacle.shp');
	file node_shape_file <- shape_file('node.shp');
	file mesh_shape_file <- shape_file('mesh.shp');
	
//	geometry shape <- envelope(mesh_shape_file);
	
	float room_width <- 60.0 #m;
	float room_length <- 40.0 #m;
	float door_width <- 3.0 #m;
	
	geometry free_space <- polygon([{0, 0}, {room_width, 0}, {room_width, room_length}, {0, room_length}, {0, 0}]);
	
	// pedestrian parameters
	bool display_free_space <- false parameter: true;
	bool display_force <- false parameter: true;
	bool display_target <- false parameter: true;
	bool display_circle_min_dist <- true parameter: true;
	
	float P_shoulder_length <- 1.0 parameter: true;
	float P_proba_detour <- 0.5 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 1.0 parameter: true ;
	float P_pedestrian_consideration_distance <- 5.0 parameter: true ;
	float P_tolerance_target <- 0.1 parameter: true;
	bool P_use_geometry_target <- true parameter: true;
	
	string P_model_type <- "advanced" among: ["simple", "advanced"] parameter: true ; 
	
	float P_A_pedestrian_SFM_advanced parameter: true <- 3.0 category: "SFM advanced" ;
	float P_A_obstacles_SFM_advanced parameter: true <- 1.9 category: "SFM advanced" ;
	float P_B_pedestrian_SFM_advanced parameter: true <- 4.0 category: "SFM advanced" ;
	float P_B_obstacles_SFM_advanced parameter: true <- 3.0 category: "SFM advanced" ;
	float P_relaxion_SFM_advanced  parameter: true <- 0.5 category: "SFM advanced" ;
	float P_gama_SFM_advanced parameter: true <- 0.35 category: "SFM advanced" ;
	float P_lambda_SFM_advanced <- 0.1 parameter: true category: "SFM advanced" ;
	float P_minimal_distance_advanced <- 0.25 parameter: true category: "SFM advanced" ;
	
	float P_n_prime_SFM_simple parameter: true <- 3.0 category: "SFM simple" ;
	float P_n_SFM_simple parameter: true <- 2.0 category: "SFM simple" ;
	float P_lambda_SFM_simple <- 2.0 parameter: true category: "SFM simple" ;
	float P_gama_SFM_simple parameter: true <- 0.35 category: "SFM simple" ;
	float P_relaxion_SFM_simple parameter: true <- 0.54 category: "SFM simple" ;
	float P_A_pedestrian_SFM_simple parameter: true <- 4.5category: "SFM simple" ;
		
	init {
		create nav_mesh from: mesh_shape_file;
		create mesh_centroid from: node_shape_file;
		create building from: obstacle_shape_file {
			free_space <- free_space - shape;
		}
		
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
		
		create walking_directly number: 1;
		create best_neighbor_mesh number: 1;
		
		create room {			
			shape <- polyline([{0, 0}, {room_width, 0}, {room_width, room_length}, {0, room_length}, {0, 0}]) 
					 - polyline([{room_width,  (room_length - door_width) / 2}, {room_width,  (room_length + door_width) / 2}]);
		}
				
		create people number: 1 {
			color <- #blue;
			target <- {room_width, room_length / 2 + 0.5};
			location <- {1.6771860484013772,27.60428474002328,0.0};
			
			obstacle_consideration_distance <-P_obstacle_consideration_distance;
			pedestrian_consideration_distance <-P_pedestrian_consideration_distance;
			shoulder_length <- P_shoulder_length;
			avoid_other <- P_avoid_other;
			proba_detour <- P_proba_detour;
			
			use_geometry_waypoint <- P_use_geometry_target;
			tolerance_waypoint<- P_tolerance_target;
			pedestrian_species <- [people];
			obstacle_species<-[building];
			
			pedestrian_model <- P_model_type;
			
		
			if (pedestrian_model = "simple") {
				A_pedestrians_SFM <- P_A_pedestrian_SFM_simple;
				relaxion_SFM <- P_relaxion_SFM_simple;
				gama_SFM <- P_gama_SFM_simple;
				lambda_SFM <- P_lambda_SFM_simple;
				n_prime_SFM <- P_n_prime_SFM_simple;
				n_SFM <- P_n_SFM_simple;
			} else {
				A_pedestrians_SFM <- P_A_pedestrian_SFM_advanced;
				A_obstacles_SFM <- P_A_obstacles_SFM_advanced;
				B_pedestrians_SFM <- P_B_pedestrian_SFM_advanced;
				B_obstacles_SFM <- P_B_obstacles_SFM_advanced;
				relaxion_SFM <- P_relaxion_SFM_advanced;
				gama_SFM <- P_gama_SFM_advanced;
				lambda_SFM <- P_lambda_SFM_advanced;
				minimal_distance <- P_minimal_distance_advanced;
			}
			
		}
		
	}

}

species people skills: [pedestrian]{
	rgb color <- rnd_color(255);
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	point target;
 	list<point> trajectory;

	reflex move {
		do walk_to target: target bounds: free_space;
		trajectory <- trajectory + [location];
	}	
	
	aspect default {
		draw circle(0.25#m) color: #blue;
		draw polyline(trajectory) color: #red;
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
			species people;
		}
	}
}