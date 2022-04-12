import numpy as np
import matplotlib.pyplot as plt
import json

def main():
    path="./build/pretty.json"
    f=open(path)
    log=json.load(f)
    gt_location=np.array(log["gt"])
    pred_location=np.array(log["pred"])
    plt.scatter(gt_location[:,0], gt_location[:,1], s=55.0, facecolors='blue', edgecolors='blue', )
    plt.scatter(pred_location[:, 0], pred_location[:, 1], s=55.0, facecolors='green', edgecolors='green', )

    plt.show()
    


if __name__ == '__main__':
    main()
    
    #test()

