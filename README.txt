General Idea for implementation:
		Task 1:
			we used yolo bounding box's center to align turtlebot and bounding box width to move turtlebot closer
			after that we publish message to control robotic arm (basic_control)
		Task 2:
			we created map of the arena to use with navigation package
			did same thing as task 1, and move it to starting position using navigation package. 
		Task 3:
			At the beginning, turtlebot rotates 270 degrees looking for bottles, if found we used task 2 to bring it back, then we repead this. If not found we go to the middle of the arena and search again.
		Task 4:
			We created new map for task 4. we predifined certain positions for turtlebot to search for bottles. Turtlebot goes to these locations in certain order until it finds all the bottles. Once we find bottle we use yolo to pickit up use navigation package to bring it back. 
