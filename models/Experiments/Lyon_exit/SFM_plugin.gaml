/**
* Name: SFMplugin
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model SFMplugin

/* Insert your model definition here */

global {
	float step <- 0.1 #s;
	
	float P_shoulder_length <- 0.3 parameter: true;
	float P_proba_detour <- 1.0 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 2.0 parameter: true ;
	float P_pedestrian_consideration_distance <- 5.0 parameter: true ;
	float P_tolerance_waypoint <- 0.5 parameter: true;
	bool P_use_geometry_waypoint <- true parameter: true;
	
	string P_model_type <- "simple" among: ["simple", "advanced", "orca"] parameter: true ; 
	
	float P_A_pedestrian_SFM_advanced parameter: true <- 0.1 category: "SFM advanced" ;
	float P_A_obstacles_SFM_advanced parameter: true <- 0.0 category: "SFM advanced" ;
	float P_B_pedestrian_SFM_advanced parameter: true <- 0.15 category: "SFM advanced" ;
	float P_B_obstacles_SFM_advanced parameter: true <- 0.3 category: "SFM advanced" ;
	float P_relaxion_SFM_advanced  parameter: true <- 0.66 category: "SFM advanced" ;
	float P_gama_SFM_advanced parameter: true <- 0.00 category: "SFM advanced" ;
	float P_lambda_SFM_advanced <- 0.1 parameter: true category: "SFM advanced" ;
	float k_SFM_advanced <- 0.0 parameter: true category: "SFM advanced" ;
	float kappa_SFM_advanced <- 0.0 parameter: true category: "SFM advanced" ;

//	float P_n_prime_SFM_simple parameter: true <- 3.0 category: "SFM simple" ;
//	float P_n_SFM_simple parameter: true <- 2.0 category: "SFM simple" ;
//	float P_lambda_SFM_simple <- 2.0 parameter: true category: "SFM simple" ;
//	float P_gama_SFM_simple parameter: true <- 0.35 category: "SFM simple" ;
//	float P_relaxion_SFM_simple parameter: true <- 0.66 category: "SFM simple" ;
//	float P_A_pedestrian_SFM_simple parameter: true <- 1.0 category: "SFM simple" ;

	float P_n_prime_SFM_simple parameter: true <- 3.0 category: "SFM simple" ;
	float P_n_SFM_simple parameter: true <- 2.0 category: "SFM simple" ;
	float P_lambda_SFM_simple <- 2.0 parameter: true category: "SFM simple" ;
	float P_gama_SFM_simple parameter: true <- 0.35 category: "SFM simple" ;
	float P_relaxion_SFM_simple parameter: true <- 0.54 category: "SFM simple" ;
	float P_A_pedestrian_SFM_simple parameter: true <- 1.0 category: "SFM simple" ;
	
	bool display_force <- false parameter: true;
	bool display_circle_min_dist <- true parameter: true;
	int nb_people <- 3000;
	geometry free_space <- copy(shape);
	
	file building_shape_file <- shape_file('../../../includes/place_des_Terreaux/ok.shp');
	file center_shape_file <- shape_file('../../../includes/place_des_Terreaux/center.shp');
	file init_space_shape_file <- shape_file('../../../includes/place_des_Terreaux/init_space.shp');
	
	geometry shape <- envelope(building_shape_file);
	
	init {
		
		create building from: building_shape_file;
		create center from: center_shape_file;
		
		create people number: nb_people {
			obstacle_consideration_distance <-P_obstacle_consideration_distance;
			pedestrian_consideration_distance <-P_pedestrian_consideration_distance;
			shoulder_length <- P_shoulder_length;
			avoid_other <- P_avoid_other;
			proba_detour <- P_proba_detour;
			use_geometry_waypoint <- P_use_geometry_waypoint;
			tolerance_waypoint <- P_tolerance_waypoint;
			
			preferred_speed <- 0.7 #m/#s;
			
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
				k_SFM <- k_SFM_advanced;
				kappa_SFM <- kappa_SFM_advanced;
			}
			
			pedestrian_species <- [people];
			obstacle_species<-[building];
			
			location <- any_location_in(first(center));
			my_target <- {16.53, 69.69};
		}
		
	}
}

species people skills: [pedestrian] {
	rgb color <- #red;
	//float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	float speed <- 1.34 #m/#s;
	point my_target ;
//	bool display_force <- false;
	
	reflex move when: my_target != nil {
		do walk_to target: my_target bounds:free_space;
		
		if (- 0.92 * location.x + location.y > 50.56) and my_target = {16.53, 69.69} {
			my_target <- one_of([{5, rnd(65.5, 69.5)}, {rnd(16.5, 21), 88}]);
			color <- #blue;
		}
		
		if distance_to(location, my_target) < 0.5 and my_target != {16.53, 69.69} {
			do die;			
		}
	}
	
	aspect base {
//		draw triangle(shoulder_length) color: color rotate: heading + 90.0;
		draw circle(shoulder_length/2) color: color;
		if  display_force {
			loop op over: forces.keys {
				if (species(agent(op)) = building ) {
					draw line([location, location + point(forces[op])]) color: #red end_arrow: 0.1;
				}
				else if ((agent(op)) = self ) {
					draw line([location, location + point(forces[op])]) color: #blue end_arrow: 0.1;
				} 
				else {
					draw line([location, location + point(forces[op])]) color: #green end_arrow: 0.1;
				}
			}
		}
	}
}


species building {
	float d <-  name = 'building9' ? 0.7 : rnd(3.5, 5.0);
	aspect default {
//		draw shape color: #yellow border: #black depth: rnd(1.5, 2.0);
		//draw shape color: rgb(209, 209, 209) border: #black depth: d;
		draw shape color: #grey border: #black depth: d;
	}
}

species center {
	aspect default {
		draw shape color: #yellow;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.05;
	output {
		display my_display {
//			species free;
			species building;
			species people aspect: base;
		}			
	}
}