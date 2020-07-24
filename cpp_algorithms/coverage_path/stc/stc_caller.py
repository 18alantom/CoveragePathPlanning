import numpy as np

def stc_caller(matrix):
    bound = matrix.shape
    r = int(bound[0]/2)
    c = int(bound[1]/2)
    grid = [[((max(abs((r-1)/2-i), abs((c-1)/2-j))+1) if(all(pt for pg in matrix[2*i:2*(i+1),2*j:2*(j+1)] for pt in pg)) else -1) for j in range(c)] for i in range(r)]
    vertices = {}
    for i in range(r):
        for j in range(c):
            if grid[i][j]>=0:
                vertices[(i, j)] = None
    s = list(vertices.keys())[0]
    pathf = []
    pathi = {}
    treef = {}
    tree = {}
    while len(vertices):
        pathi[s] = None
        del vertices[s]
        if s not in tree: tree[s] = np.full(4, 0)
        grid[s[0]][s[1]] = 0
        tmp = [0, 0, 0, 0]
        tmp[0] = grid[s[0]-1][s[1]] if s[0]-1>=0 and grid[s[0]-1][s[1]]!=-1 else -1
        tmp[1] = grid[s[0]][s[1]+1] if s[1]+1<c and grid[s[0]][s[1]+1]!=-1 else -1
        tmp[2] = grid[s[0]+1][s[1]] if s[0]+1<r and grid[s[0]+1][s[1]]!=-1 else -1
        tmp[3] = grid[s[0]][s[1]-1] if s[1]-1>=0 and grid[s[0]][s[1]-1]!=-1 else -1
        grt = max(tmp)
        if grt>0:
            ind = tmp.index(grt)
            s = (s[0]-1, s[1]) if ind==0 else (s[0], s[1]+1) if ind==1 else (s[0]+1, s[1]) if ind==2 else (s[0], s[1]-1)
        else:
            pathf.append(pathi)
            pathi = {}
            treef.update(tree)
            tree = {}
            if len(vertices.keys()):
                s = list(vertices.keys())[0]
    for p in range(len(pathf)):
        path = list(pathf[p].keys())
        for i in range(1, len(path)):
            ind = 0 if path[i-1][0]-1==path[i][0] else 1 if path[i-1][1]+1==path[i][1] else 2 if path[i-1][0]+1==path[i][0] else 3
            treef[path[i-1]][ind] = 1
            treef[path[i]][(ind+2)%4] = 1
            
    a = np.array([len(i) for i in pathf])
    b = np.argsort(a)
    allPaths = [list(pathf[i].keys()) for i in range(len(pathf))]
    edgePath = np.array([i for i in range(len(pathf))])
    ejPath = []

    def whichPath(ver):
        for i in range(len(pathf)):
            if ver in pathf[i]: return i
        return 0

    def mergePath(ind1, v1, v2):
        ind2 = edgePath[whichPath(v2)]
        if len(pathf[ind1])>len(pathf[ind2]):
            pathf[ind1].update(pathf[ind2])
            edgePath[edgePath==ind2] = ind1
        else:
            pathf[ind2].update(pathf[ind1])
            edgePath[edgePath==ind1] = ind2
        ejPath.append([v1, v2])

    for index in range(len(b)-1):
        i = b[index]
        for j in allPaths[i]:
            x, y = j
            if x-1>=0 and grid[x-1][y]!=-1 and (x-1, y) not in pathf[edgePath[i]]:
                mergePath(edgePath[i], j, (x-1, y))
            if y-1>=0 and grid[x][y-1]!=-1 and (x, y-1) not in pathf[edgePath[i]]:
                mergePath(edgePath[i], j, (x, y-1))
            if x+1<r and grid[x+1][y]!=-1 and (x+1, y) not in pathf[edgePath[i]]:
                mergePath(edgePath[i], j, (x+1, y))
            if y+1<c and grid[x][y+1]!=-1 and (x, y+1) not in pathf[edgePath[i]]:
                mergePath(edgePath[i], j, (x, y+1))

    for p in range(len(ejPath)):
        path = ejPath[p]
        ind = 0 if path[0][0]-1==path[1][0] else 1 if path[0][1]+1==path[1][1] else 2 if path[0][0]+1==path[1][0] else 3
        treef[path[0]][ind] = 1
        treef[path[1]][(ind+2)%4] = 1
        
    finalPathf = []
    finalPath = []
    for i in set(edgePath):
        vertex = tuple(2*a for a in (list(pathf[i].keys())[0]))
        while True:
            finalPath.append(vertex)
            vertex_grid = tuple(int(a/2) for a in vertex)
            if vertex[0]%2==0 and vertex[1]%2==0:
                if treef[vertex_grid][0]:
                    vertex = (vertex[0]-1, vertex[1])
                else:
                    vertex = (vertex[0], vertex[1]+1)
            elif vertex[0]%2==0:
                if treef[vertex_grid][1]:
                    vertex = (vertex[0], vertex[1]+1)
                else:
                    vertex = (vertex[0]+1, vertex[1])
            elif vertex[1]%2==0:
                if treef[vertex_grid][3]:
                    vertex = (vertex[0], vertex[1]-1)
                else:
                    vertex = (vertex[0]-1, vertex[1])
            else:
                if treef[vertex_grid][2]:
                    vertex = (vertex[0]+1, vertex[1])
                else:
                    vertex = (vertex[0], vertex[1]-1)
            if vertex==finalPath[0]: 
                finalPath.append(vertex)
                break
        finalPathf.append(finalPath)
        finalPath = []
    return finalPathf