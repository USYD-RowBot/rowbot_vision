# KNN.py
import numpy as np

## Good old unionfind

def query(con, x):
    if (con[x] != x)con[x] = query(con,x);
    return con[x];
    
def update(con, x, y):
    con[query(x)]=query(y)
    return con; # just in case

def connected(con,x,y):
    return query(con,x)==query(con,y);

def runKNN(vertices):
    connections=range(n)

    vertexDistList=[];
    # Calculate all vertex-vertex distances.
    for c,i in enumerate(vertices):
        for d,j in enumerate(vertices):
            if not c==d:
                vertexDistList.append((c,d,np.sqrt((i[0]-j[0])*(i[0]-j[0])+(i[1]-j[1])*(i[1]-j[1]))))
    vertexDistList=sorted(vertexDistList, key=lambda i: i[2]);
            
    # At each iteration:

