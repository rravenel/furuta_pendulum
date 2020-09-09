import json

PATH_DATA = "../data/current_data.json"
TARGET = "target"
CURRENT = "i"

def loadData():
    with open(PATH_DATA, 'r') as f:
        data = json.load(f)
    return data

def formatFloat(data):
    d = data[0]
    current = d[CURRENT]
    
    if 1 > len(current):
        print("No current data in first sample; exiting.")
        exit()

    datum = current[0]
    if isinstance(datum, float):
        print("is float")
        return True
    print("is not float")
    return False

def averageCurrent(current, floatType=False):
    buf = []
    for c in current:
        if not floatType:
            c = c[1]
        buf.append(c)
        
    return sum(buf)/float(len(buf))

def main():
    data = loadData()
    if 1 > len(data):
        print("No data; exiting.")
        exit()
    
    isFloat = formatFloat(data)
    
    buf_err = []
    err_min = 1000
    err_max = 0
    
    for i in range(len(data)):
        d = data[i]
        target = d[TARGET]
        current = d[CURRENT]
        
        print(target)
        
        if 1 > len(current):
            print("Empty data set - skipping.")
            continue
        
        ave = averageCurrent(current, isFloat)
        err = target - ave
        buf_err.append(err)
        
        if abs(err) < err_min:
            err_min = abs(err)
        if abs(err) > err_max:
            err_max = abs(err)
        
        if i > len(data)/2 and i < len(data)/2 + 20:
            print("target: %.2f\tave: %.2f\terr: %.2f" % (target, ave, err))
    
    print("err_min: %.2f\terr_max: %.2f" % (err_min, err_max))
            
    
if __name__ == "__main__":
    main()