/**
* Name: testdbscan
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model testdbscan
import 'helpers/detection.gaml'
/* Insert your model definition here */

global {
	list<list<float>> data;
	init {
//		int N <- 100;
//		loop i from: 1 to: N {
//			list<float> temp <- list_with(2, rnd(50.0));
//			data <- data + [temp];
//		}
//		write data;
//		list<list> var0 <- dbscan ([[0.0, 0.0], [0.1, 0.1], [0.2, 0.2], [50, 50], [51, 51], [50.5, 50.5], [100, 100]], 10, 2); // var0 equals [[0,1,2,3]]
		
		file my_csv_file <- csv_file("../includes/test.csv",",");
		matrix points <- matrix(my_csv_file);
		
		list<list<float>> my_data;
		loop i from: 0 to: points.rows - 1 {
			create my_point {
				x <- float(points[0, i]);
				y <- float(points[1, i]);
			}
			list<float> p <- [float(points[0, i]), float(points[1, i])]; 
			my_data <- my_data + [p];
		}
		
		list<float> bound <- detect_dense_region(my_data, 1.0, 3, 80, 9.0);

		create my_boundary {
			x_min <- bound[0];
			x_max <- bound[1];
			y_min <- bound[2];
			y_max <- bound[3];
		}
		
	}
	
//	reflex run_dbscan {
////		write data;
//		int N <- 1000;
//		data <- [];
//		loop i from: 1 to: N {
//			list<float> temp <- list_with(2, rnd(100.0));
//			data <- data + [temp];
//		}
//		list<list<int>> var0 <- dbscan (data, 1.0, 2); // var0 equals [[0,1,2,3]]
////		write data;
////		write var0;
//		write "--";
//		list<int> len;
//		loop d over: var0 {
//			len <- len + length(d);
//		}
//		write len;
//		list var1 <- [1, 7, 8, 3, 5, 8, 9, 12] sort_by (each); 
//		write var1;
////		write length(var0);
////		write "--";
////		loop idx over: var0[0] {
////			write data[idx];
////		}
//
//	}

}

species my_point {
	float x;
	float y;
	aspect default {
		draw circle(0.1) at: {x, y} color: #blue;
	}
}

species my_boundary {
	float x_min;
	float x_max;
	float y_min;
	float y_max;
	
	aspect default {
		draw polygon([{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}]) color: #red;
	}
}

experiment exp {
	output {
		display my_display {
			species my_boundary;
			species my_point;
			
		}
	}
}