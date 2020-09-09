import json
import os

TARGET = "target"
CURRENT = "i"


model_path = "current_network_2x3_b.h5"
data_name_prefix = "current_data_3_b_"
data_dir = "../data"

def loadData():

    ## load data ##

    data1= []

    files = os.listdir(data_dir)
    for name in files:
        if not name.startswith(data_name_prefix):
            continue
        with open(data_dir +'/' + name, 'r') as file:
            data = json.load(file)
            print("Sample size: %d" % len(data))
            data1 += data
    print("Data size: %d" % len(data1))

    return data1

def main():
    data = loadData()
    if 1 > len(data):
        print("No data; exiting.")
        exit()
    
    buf = []
    
    count = 0
    target_last = data[0][TARGET]
    for i in range(len(data)):
        d = data[i]
        target = d[TARGET]
        
        if target != target_last:
            target_last = target
            buf.append(count)
            count = 0
            
        count += 1
        
    print("Average samples per step: %.3f" % (sum(buf)/float(len(buf))))
            
    
if __name__ == "__main__":
    main()