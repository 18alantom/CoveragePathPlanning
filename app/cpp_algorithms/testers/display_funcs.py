import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from cpp_algorithms.common_helpers import imshow
from cpp_algorithms.fuel_path import splice_paths

def printer(metrics):
    for m in metrics:
        print(f"{m.rjust(20)} : {metrics[m]}")

def path_show(area_map, coverage_path,fuel_paths=None, \
              dist_map=None, figsize=(7,7),s=7, alpha=0.5,
              cmap="Greys_r"):
    """
    Displays `area_map` with the path 
    taken by the drone overlaid ontop of 
    it.
    
    PARAMETERS
    ---
    area_map : -1 (obstacle), 0 valued 2-dim ndarray
    coverage_path : area coverage path [(x1,y1),..., (xm,ym)]
    fuel_paths : detour paths to refuel [[(x1,y1),..., (xn,yn)],...,[...]]
    dist_map : path showing dist gradients to the fuel points
    s : path marker size
    alpha : path marker alpha
    """
    x,y = np.array(coverage_path).T
    if dist_map is not None:
        imshow(dist_map,figsize,cmap=cmap)
    else:
        imshow(area_map,figsize,cmap=cmap)
    sd = 3 
    ad = 0.2
    plt.scatter(y,x,color="orange", s=s,alpha=alpha)
    plt.scatter(y[0],x[0],color="green", s=s+sd,alpha=alpha+ad)
    plt.scatter(y[-1],x[-1],color="red", s=s+sd,alpha=alpha+ad)

    if fuel_paths is not None:
        for fuel_path in fuel_paths:
            x,y = np.array(fuel_path).T
            plt.scatter(y,x,color="lightblue", s=s,alpha=alpha)
            plt.scatter(y[0],x[0],color="yellow", s=s+sd,alpha=alpha+ad)
            plt.scatter(y[-1],x[-1],color="cyan", s=s+sd,alpha=alpha+ad)


def path_animate(values, interval=10, repeat=False):
    disp_map = values["dist_map"]
    if disp_map is None:
        disp_map = values["area_map"]

    coverage_path = values['coverage_path']
    start_point = values["start_point"]
    end_point = values["end_point"]

    fuel_points = values["fuel_points"]
    if fuel_points is not None:
        fuel_paths = values["fuel_paths"]
        detour_idx = values["detour_idx"]
        full_path,detour_se_idx =  splice_paths(coverage_path, fuel_paths, detour_idx)
        det_se_flat = np.array(detour_se_idx).flatten()
    else:
        full_path = np.array(coverage_path)
        detour_se_idx = []
        det_se_flat = []

    frames = full_path.shape[0]

    fig, ax = plt.subplots(figsize=(8,8))
    ax.imshow(disp_map, interpolation="none", cmap="cividis")
    ax.axis('off')

    x,y = np.array(full_path).T

    features = {
        "c_count":-1,
        "f_count":0,
        "f_num": -1,
        "fuelling": False,
        "text" : ax.text(0,-2," - "),
        "c_path" : ax.plot([],[], color="#FF69B4", alpha=0.8)[0],
        "f_path" : [ax.plot([],[], color="#BBDEFB", alpha=0.8)\
                    [0] for _ in range(len(detour_se_idx))]
    }

    if fuel_points is not None:
        f,g = np.array(fuel_points).T
        ax.scatter(g,f, s=10, color="#FFF")
    ax.scatter(start_point[1],start_point[0], s=12, color="#b2ebf2")
    ax.scatter(end_point[1],end_point[0], s=12, color="#FF3D00")

    def next_frame(i):
        is_fu = features["fuelling"]
        if not is_fu: 
            features["c_count"] = features["c_count"] + 1
        else:
            features["f_count"] = features["f_count"] + 1

        if i in det_se_flat:
            if not is_fu:
                ax.scatter(y[i],x[i], s=12, color="#ffca28")
                features["fuelling"] = True
                features["f_num"] += 1
            else:
                features["fuelling"] = False

        mode = "fuelling" if is_fu else "mapping"
        features["text"].set_text(f"{mode} :"+\
                f": total path left : {frames - i - 1} :"+\
                f": f_count :{features['f_count']} :"+\
                f": c_count :{features['c_count']} :"+\
                f": num_refuels:{features['f_num']+1}")

        if features["fuelling"]:
            f_num = features["f_num"]
            y_,x_ = features["f_path"][f_num].get_data()
            x_ = [*x_, x[i]]
            y_ = [*y_, y[i]]
            features["f_path"][f_num].set_data(y_,x_)
        else:
            y_,x_ = features["c_path"].get_data()
            x_ = [*x_, x[i]]
            y_ = [*y_, y[i]]
            features["c_path"].set_data(y_,x_)
    return FuncAnimation(fig, next_frame, frames=frames, interval=interval, repeat=repeat);