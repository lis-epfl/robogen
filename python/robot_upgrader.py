import json
import sys

       
if __name__ == "__main__":

    if len(sys.argv) < 2 :
        print "Usage: python robot_upgrader.py input.json [output.json]"
	print "If output not provided, will overwrite input"
        exit()
    
    robot = json.load(open(sys.argv[1],"r"))
    if len(sys.argv) == 2 :
        output = open(sys.argv[1],"w")
    else :
        output = open(sys.argv[2],"w")
    

    if "brain" in robot :
        if "neuron" in robot["brain"]: 
            for neuron in robot["brain"]["neuron"] :
                if (not "bias" in neuron) and ("biasWeight" in neuron) :
                    neuron["bias"] = neuron["biasWeight"]
                    del neuron["biasWeight"]
                if not "type" in neuron :
                    if neuron["layer"] == "input" :
                        neuron["type"] = "simple"
                    else :
                        neuron["type"] = "sigmoid"
                if not "gain" in neuron :
                    neuron["gain"] = 1.0
    
    json.dump(robot, output, indent=1)
    output.close()
