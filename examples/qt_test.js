{
	// here we define a variable for record keeping
	distances : [],    
	// optional function called at the beginning of the simulation
	setupSimulation: function() {
		this.startPos = this.getRobot().getCoreComponent().getRootPosition();
		return true;
	},

	afterSimulationStep : function() {
		/*var sensors = this.getRobot().getSensors();
		var tmp = ""
		for (var i = 0; i < sensors.length; i++) {
			tmp += sensors[i].getLabel() + " " + sensors[i].read() + " ";
		}
		print(tmp);*/

		return true;
	},

	// optional function called at the end of the simulation
	endSimulation: function() {
		// Compute robot ending position from its closest part to the origin
		var minDistance = Number.MAX_VALUE;
		
		var bodyParts = this.getRobot().getBodyParts();
		print(bodyParts.length + " body parts");
		for (var i = 0; i < bodyParts.length; i++) {
			var xDiff = (bodyParts[i].getRootPosition().x - this.startPos.x);
			var yDiff = (bodyParts[i].getRootPosition().y - this.startPos.y);
			var dist = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));		
			if (dist < minDistance) {
				minDistance = dist;
			}
		}

		this.distances.push(minDistance);
		return true;
	},

	// the one required method... return the fitness!
	// here we return minimum distance travelled across evaluations
	getFitness: function() {
		fitness = this.distances[0];
		for (var i=1; i<this.distances.length; i++) {
			if (this.distances[i] < fitness)
				fitness = this.distances[i];
		}

		//print(fitness);
		return fitness;
	},

}
