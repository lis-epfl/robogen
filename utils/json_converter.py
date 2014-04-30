import json
import sys


def get_root(robot) :
    for part in robot["body"]["part"] :
        if part["root"] :
            return part    

def get_part(id_, body) :
    for part in body["part"] :
        if part["id"] == id_ :
            return part

def fix_param(val) :
    return round(val * 10000)/10000.0


def write_part(output, part, body, indentation_level=0) :
    for i in range(indentation_level) :
        output.write("\t")
    if part["root"] :
        output.write("0")
    else :
        for connection in body["connection"] :
            if connection["dest"] == part["id"] :
                output.write(str(connection["srcSlot"]))
    output.write(" ")
    output.write(part["type"])
    output.write(" ")
    output.write(str(part["id"]))
    output.write(" ")
    output.write(str(part["orientation"]))
    if "evolvableParam" in part :
        for param in part["evolvableParam"] :
            output.write(" ")
            output.write(str(fix_param(param["paramValue"])))
    output.write("\n")
    for connection in body["connection"] :
        if connection["src"] == part["id"] :            
            write_part(output, get_part(connection["dest"], body), body, indentation_level+1)
        


if len(sys.argv) < 3 :
    print "Usage: python json_converter.py input.json output.txt"
    exit()

robot = json.load(open(sys.argv[1],"r"))

output = open(sys.argv[2], "w")

root = get_root(robot)
write_part(output, root, robot["body"])
output.write("\n") 


for connection in robot["brain"]["connection"] :
    #print connection["weight"]
    output.write(connection["src"].split("-")[0])
    output.write(" ")
    output.write(connection["src"].split("-")[1])
    output.write(" ")
    output.write(connection["dest"].split("-")[0])
    output.write(" ")
    output.write(connection["dest"].split("-")[1])
    output.write(" ")
    output.write(str(connection["weight"]))
    output.write("\n")
output.write("\n")
for neuron in robot["brain"]["neuron"] :
    if neuron["layer"] != "input" :
        output.write(neuron["id"].split("-")[0])
        output.write(" ")
        output.write(neuron["id"].split("-")[1])
        output.write(" ")
        output.write(str(neuron["biasWeight"]))
        output.write("\n")


