{
    // here we define a variable for record keeping
    velocities : [],    
    deltaVelocities : [],
    maxIrVals : [],
    fitnesses : [],

    setupSimulation: function() {
	this.velocities = [];
	this.deltaVelocities = [];
	this.maxIrVals = [];
	return true;
    },

    // optional function called after each step of simulation
    afterSimulationStep: function() {
	var sensors = this.getRobot().getSensors();
	var maxIr = 0

	for (var i = 0; i < sensors.length; i++) {
		if (/^Distance/.test(sensors[i].getLabel())) {
			if (sensors[i].read() > maxIr)
				maxIr = sensors[i].read();
		}			

	}


	this.maxIrVals.push(maxIr);

	var motors = this.getRobot().getMotors();

	var meanVelocity = (motors[1].getVelocity() - motors[0].getVelocity()) / (2.0 * 2.0 * Math.PI);
	meanVelocity = (meanVelocity + 1)/2.0;
	//console.log(meanVelocity);

	this.velocities.push(meanVelocity);



	var deltaVelocity = Math.abs(motors[1].getVelocity() + motors[0].getVelocity()) / (2.0 * 2.0 * Math.PI);

	this.deltaVelocities.push(deltaVelocity);

	return true;
    },

    // optional function called at the end of the simulation
    endSimulation: function() {
	var sum = 0;
	for (var i = 0; i < this.maxIrVals.length; i++) {
		sum += this.velocities[i] * ( 1 - Math.sqrt(this.deltaVelocities[i]) ) * ( 1 - this.maxIrVals[i] )
	}
		
	
	this.fitnesses.push(sum);

	return true;
    },
    // the one required method... return the fitness!
    getFitness: function() {
	var fitness = this.fitnesses[0];
	for (var i= 1; i < this.fitnesses.length; i++) {
		if (this.fitnesses[i] < fitness)
			fitness = this.fitnesses[i];
	}
	return fitness;
    },

}
