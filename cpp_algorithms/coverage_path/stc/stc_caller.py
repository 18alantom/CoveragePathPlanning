import numpy as np

def stc_caller(matrix):
    bound = matrix.shape
    
    #Step 2
    r = int(bound[0]/2)
    c = int(bound[1]/2)
    grid = [[((max(abs((r-1)/2-i), abs((c-1)/2-j))+1) if(all(pt for pg in matrix[2*i:2*(i+1),2*j:2*(j+1)] for pt in pg)) else -1) for j in range(c)] for i in range(r)]
    vertices = {}
    obstacles = {}
    for i in range(r):
        for j in range(c):
            if grid[i][j]>=0:
                vertices[(i, j)] = None
            elif not all(not matrix[2*i+ri][2*j+cj] for ri in range(2) for cj in range(2)):
                obstacles[(i, j)] = []
                for ri in range(2):
                    for jc in [ri, abs(ri-1)]:
                        if matrix[2*i+ri][2*j+jc]:
                            obstacles[(i, j)].append(1)
                            matrix[2*i+ri][2*j+jc] = 191
                        else:
                            obstacles[(i, j)].append(0)

    s = list(vertices.keys())[0]
    # Step 3
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

    # Step 4
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

    # Step 5
    def getNextPoint(vertex, vertex_grid):
        if vertex[0]%2==0 and vertex[1]%2==0:
            if treef[vertex_grid][0]:
                return (vertex[0]-1, vertex[1])
            else:
                return (vertex[0], vertex[1]+1)
        elif vertex[0]%2==0:
            if treef[vertex_grid][1]:
                return (vertex[0], vertex[1]+1)
            else:
                return (vertex[0]+1, vertex[1])
        elif vertex[1]%2==0:
            if treef[vertex_grid][3]:
                return (vertex[0], vertex[1]-1)
            else:
                return (vertex[0]-1, vertex[1])
        else:
            if treef[vertex_grid][2]:
                return (vertex[0]+1, vertex[1])
            else:
                return (vertex[0], vertex[1]-1)

    finalPathf = {}
    k = 0
    for i in set(edgePath):
        finalPath = {}
        vertex = tuple(2*a for a in (list(pathf[i].keys())[0]))
        svertex = vertex
        pvertex = vertex
        vertex = getNextPoint(vertex, tuple(int(a/2) for a in vertex))
        nvertex = getNextPoint(vertex, tuple(int(a/2) for a in vertex))
        finalPath[vertex] = {pvertex:nvertex}
        while vertex!=svertex:
            pvertex = vertex
            vertex = nvertex
            nvertex = getNextPoint(vertex, tuple(int(a/2) for a in vertex))
            finalPath[vertex] = {pvertex:nvertex}
        finalPathf[k] = finalPath
        k += 1

    # Step 6
    def nextPoint(ap, i):
        if i==0:
            return (ap[0], ap[1]-1) if ap[1]-1>=0 else None
        elif i==1:
            return (ap[0]-1, ap[1]) if ap[0]-1>=0 else None
        elif i==2:
            return (ap[0], ap[1]+1) if ap[1]+1<bound[1] else None
        else:
            return (ap[0]+1, ap[1]) if ap[0]+1<bound[0] else None

    def actualPoint(og, i):
        if i==0:
            return (2*og[0], 2*og[1])
        elif i==1:
            return (2*og[0], 2*og[1]+1)
        elif i==2:
            return (2*og[0]+1, 2*og[1]+1)
        else:
            return (2*og[0]+1, 2*og[1])

    def ifInPath(a):
        if a==None: return -1
        if matrix[a]==0: return -2
        for i in finalPathf:
            if a in finalPathf[i]: return i
        return -1

    def findBefore(p_, pn2):
        return list(finalPathf[pn2][p_])[0]

    def insert2(pi, po, pp, a, b):
        if pi not in finalPathf[pp][po]: return False
        pi_p = None
        for i in finalPathf[pp][pi]:
            if finalPathf[pp][pi][i]==po:
                pi_p = i
                break
        if pi_p==None:
            print("Error", pi, po, pp)
            return False
        finalPathf[pp][pi][pi_p] = a
        finalPathf[pp][a] = {pi:b}
        finalPathf[pp][b] = {a:po}
        finalPathf[pp][po][b] = finalPathf[pp][po][pi]
        del finalPathf[pp][po][pi]
        return True

    def insert1(a, poa, b):
        pa = findBefore(a, poa)
        finalPathf[poa][a][b] = finalPathf[poa][a][pa]
        finalPathf[poa][a][pa] = b

    def joinPath(a, poa, drc):
        b = nextPoint(a, drc)
        pob = ifInPath(b)
        if poa==pob or pob<0: return poa
        insert1(a, poa, b)
        insert1(b, pob, a)
        if len(finalPathf[poa])<len(finalPathf[pob]):
            potmp = poa
            poa = pob
            pob = potmp
        finalPathf[poa].update(finalPathf[pob])
        del finalPathf[pob]
        return poa

    for ogrid in obstacles:
        for i in range(3, -1, -1):
            if obstacles[ogrid][i]!=1 or obstacles[ogrid][i-1]!=1: continue
            a = actualPoint(ogrid, i)
            a_ = nextPoint(a, i)
            pn1 = ifInPath(a_)
            if pn1<0: continue
            b = actualPoint(ogrid, i-1)
            b_ = nextPoint(b, i)
            if b_==None or b_ not in finalPathf[pn1]: continue
            if not insert2(a_, b_, pn1, a, b): continue
            pn1 = joinPath(a, pn1, (i+1)%4)
            obstacles[ogrid][i] = -1
            obstacles[ogrid][i-1] = -1

    # Step 7
    def addInFinalPath(tmpDict):
        lastEntry = list(finalPathf)[-1]
        finalPathf[lastEntry+1] = tmpDict
        return lastEntry+1

    for ogrid in list(obstacles):
        for i in range(3, -1, -1):
            flaga = 0
            flagb = 0
            if obstacles[ogrid][i]==1:
                a = actualPoint(ogrid, i)
                a_ = nextPoint(a, i)
                pa_ = ifInPath(a_)
                if a_!=None and pa_==-1:
                    ao = nextPoint(a, (i+1)%4)
                    pao = ifInPath(ao)
                    if ao!=None and pao>=0:
                        a_o = nextPoint(a_, (i+1)%4)
                        pa_o = ifInPath(a_o)
                        if a_o!=None and pa_o==pao and insert2(ao, a_o, pao, a, a_):
                            flaga = 2
                        else:
                            flaga = 1
                    else:
                        flaga = 1

            if obstacles[ogrid][i-1]==1:
                b = actualPoint(ogrid, i-1)
                b_ = nextPoint(b, i)
                pb_ = ifInPath(b_)
                if b_!=None and pb_==-1:
                    bo = nextPoint(b, i-1)
                    pbo = ifInPath(bo)
                    if bo!=None and pbo>=0:
                        b_o = nextPoint(b_, i-1)
                        pb_o = ifInPath(b_o)
                        if b_o!=None and pb_o==pbo and insert2(b_o, bo, pbo, b_, b):
                            flagb = 2
                            if flaga==1 and not insert2(b_, b, pbo, a_, a):
                                flaga=0
                            elif flaga==2:
                                if pbo!=pao:
                                    finalPathf[pao][a][ao] = b
                                    finalPathf[pbo][b][a] = bo
                                    finalPathf[pbo][b][b_o] = a_
                                    finalPathf[pao][a_][b_] = ao
                                    if len(finalPathf[pbo])>len(finalPathf[pao]):
                                        tmpo = pbo
                                        pbo = pao
                                        pao = tmpo
                                    finalPathf[pao].update(finalPathf[pbo])

                                    del finalPathf[pbo]
                        else:
                            flagb = 1
                    else:
                        flagb = 1
                    if flagb==1:
                        if flaga==1:
                            tmpcp = {}
                            tmpcp[a] = {a_:b}
                            tmpcp[b] = {a:b_}
                            tmpcp[b_] = {b:a_}
                            tmpcp[a_] = {b_:a}
                            pao = addInFinalPath(tmpcp)
                        elif flaga==2 and not insert2(a, a_, pao, b, b_):
                            flagb=0
            if flaga==2 or (flaga==1 and flagb!=0):
                obstacles[ogrid][i] = -1
                obstacles[nextPoint(ogrid, i)][(i+1)%4] = -1
            if flagb==2 or (flagb==1 and flaga!=0):
                obstacles[ogrid][i-1] = -1
                obstacles[nextPoint(ogrid, i)][(i+2)%4] = -1
            if flaga==2 and flagb==0:
                #ind a backtr
                pao = joinPath(a, pao, (i+2)%4)
                joinPath(a_, pao, i)
            elif flagb==2 and flaga==0:
                #ind b backtr
                pbo = joinPath(b_, pbo, i)
                joinPath(b, pbo, (i+2)%4)
            elif flaga!=0 and flagb!=0:
                if flaga==2 and flagb==1:
                    #b on a back
                    pao = joinPath(a, pao, (i+2)%4)
                    pao = joinPath(b, pao, i-1)
                    pao = joinPath(b, pao, (i+2)%4)
                    pao = joinPath(b_, pao, i)
                    pao = joinPath(b_, pao, i-1)
                    joinPath(a_, pao, i)
                elif flagb==2 and flaga==1:
                    #a on b back
                    pbo = joinPath(b_, pbo, i)
                    pbo = joinPath(a_, pbo, i)
                    pbo = joinPath(a_, pbo, (i+1)%4)
                    pbo = joinPath(a, pbo, (i+1)%4)
                    pbo = joinPath(a, pbo, (i+2)%4)
                    joinPath(b, pbo, (i+2)%4)
                elif flaga==2 and flagb==2:
                    #a and b bck
                    pao = joinPath(a, pao, (i+2)%4)
                    pao = joinPath(b, pao, (i+2)%4)
                    pao = joinPath(b_, pao, i)
                    joinPath(a_, pao, i)
                else:
                    pao = joinPath(a, pao, (i+1)%4)
                    pao = joinPath(a, pao, (i+2)%4)
                    pao = joinPath(b, pao, (i+2)%4)
                    pao = joinPath(b, pao, i-1)
                    pao = joinPath(b_, pao, i-1)
                    pao = joinPath(b_, pao, i)
                    pao = joinPath(a_, pao, i)
                    joinPath(a_, pao, (i+1)%4)

            elif flaga==1 and i==0:
                au = nextPoint(a, 1)
                pau = ifInPath(au)
                if au!=None and pau==-1:
                    ad = nextPoint(a_, 1)
                    pad = ifInPath(ad)
                    if ad!=None and pad==-1:
                        tmpcp = {}
                        tmpcp[a_] = {a:ad}
                        tmpcp[ad] = {a_:au}
                        tmpcp[au] = {ad:a}
                        tmpcp[a] = {au:a_}
                        pao = addInFinalPath(tmpcp)
                        obstacles[ogrid][i] = -1
                        obstacles[nextPoint(ogrid, 0)][1] = -1
                        obstacles[nextPoint(ogrid, 1)][3] = -1
                        obstacles[nextPoint(nextPoint(ogrid, 0), 1)][2] = -1

                        tmpcp = list(tmpcp)
                        for ij in range(4):
                            pao = joinPath(tmpcp[ij], pao, ij)
                            pao = joinPath(tmpcp[(ij+1)%4], pao, ij)
        if all(obstacles[ogrid][i]!=1 for i in range(4)): del obstacles[ogrid]

    # Step 8
    while True:
        flag = True
        for ogrid in list(obstacles):
            for i in range(3, -1, -1):
                if obstacles[ogrid][i]==1:
                    a = actualPoint(ogrid, i)
                    tmpflag = 0
                    for j in range(4):
                        npt = nextPoint(a, j)
                        ppt = ifInPath(npt)
                        if npt!=None and ppt==-2: tmpflag += 1
                        elif npt!=None and ppt>=0:
                            pnpt = findBefore(npt, ppt)
                            finalPathf[ppt][npt][a] = finalPathf[ppt][npt][pnpt]
                            finalPathf[ppt][npt][pnpt] = a
                            finalPathf[ppt][a] = {npt:npt}
                            for k in range(j+1, 4):
                                ppt = joinPath(a, ppt, k)
                            obstacles[ogrid][i] = -1
                            flag = False
                            break
                    if tmpflag==4: obstacles[ogrid][i] = -1
            if all(obstacles[ogrid][i]!=1 for i in range(4)): del obstacles[ogrid]
        if len(obstacles)==0 or flag: break

    # Step 9
    coverage_path = []
    for ind in finalPathf:
        i = finalPathf[ind]
        pvertex = list(i.keys())[0]
        vertex = i[pvertex][list(i[pvertex].keys())[0]] if type(i[pvertex])==dict else i[pvertex]

        while True:
#             matrix[vertex] = 127
            coverage_path.append(vertex)
            if vertex not in i or pvertex not in i[vertex]: break
            psvertex = pvertex
            pvertex = vertex
            vertex = i[pvertex][psvertex]
            del i[pvertex][psvertex]
            if len(i[vertex])==0:
                del i[vertex]
    return coverage_path