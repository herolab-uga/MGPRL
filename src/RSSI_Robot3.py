#!/usr/bin/env python3
import rospy, numpy as np, GPy, math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from scipy.ndimage import maximum_filter, label, find_objects
from scipy.spatial import ConvexHull
from numpy.linalg import svd, norm

rospy.init_node("rssi_robot_3")
pub = rospy.Publisher("robot3_ap_data", Float32MultiArray, queue_size=10)
sub1 = rospy.Subscriber("robot1_ap_data", Float32MultiArray, lambda m: data_cb(1, m))
sub2 = rospy.Subscriber("robot2_ap_data", Float32MultiArray, lambda m: data_cb(2, m))
pose_pub = rospy.Publisher("robot3_relative", Float32MultiArray, queue_size=10)
robot_pose = None
other_data = {}

def pose_cb(msg):
    global robot_pose
    robot_pose = np.array([msg.pose.position.x, msg.pose.position.y])
rospy.Subscriber("robot3_pose", PoseStamped, pose_cb)
env_size=10; grid_size=50
xg = np.linspace(0,env_size,grid_size); yg = np.linspace(0,env_size,grid_size)
xx,yy = np.meshgrid(xg,yg); grid_pts = np.column_stack([xx.ravel(),yy.ravel()])
num_aps=4
X_meas=[]; Y_meas=[[] for _ in range(num_aps)]
def simulate_rssi(pos, ap):
    d = np.linalg.norm(pos-ap)
    if d<1.0: d=1.0
    pl = -20.0*math.log10(d)
    return pl + np.random.normal(0,6) + np.random.normal(0,1)
AP_true = np.array([[2,2],[8,2],[2,8],[8,8]])
rate = rospy.Rate(0.2)


def gp_model():
    X=np.array(X_meas)
    models=[]; pred_means=[]; pred_stds=[]
    for i in range(num_aps):
        y = np.array(Y_meas[i]).reshape(-1,1)
        spatial_kernel = GPy.kern.RBF(input_dim=2, active_dims=[0, 1], lengthscale=2.0)
        coreg = GPy.kern.Coregionalize(input_dim=1, output_dim=3, rank=2, active_dims=[2])
        kernel_mo = spatial_kernel * coreg
        model_mo = GPy.models.GPRegression(X, y, kernel_mo)
        model_mo.Gaussian_noise.variance = 0.5
        model_mo.optimize_restarts(num_restarts=10, max_iters=1000, messages=True)
        mu, var = m.predict(grid_pts)
        pred_means.append(mu.reshape(grid_size,grid_size))
        pred_stds.append(np.sqrt(var.reshape(grid_size,grid_size)))
    return models, pred_means, pred_stds

def detect_maxima(field):
    gmax = np.unravel_index(np.argmax(field, axis=None), field.shape)
    global_pt = np.array([xg[gmax[1]], yg[gmax[0]]])
    local = maximum_filter(field, size=3)==field
    lab, nf = label(local)
    objs = find_objects(lab)
    cands=[]
    for j in range(nf):
        sl = objs[j]
        reg = field[sl]
        lmax = np.unravel_index(np.argmax(reg, axis=None), reg.shape)
        i0 = lmax[0]+sl[0].start; j0 = lmax[1]+sl[1].start
        cand = np.array([xg[j0], yg[i0]])
        cands.append(cand)
    return global_pt, cands

def weight_candidate(std_val, eps=1e-3, alpha=1.5):
    return max(eps, alpha/(1+std_val))

def svd_alignment(P, Q, W):
    W_mat = np.diag(W)
    mu_P = np.sum(P*W[:,None], axis=0)/np.sum(W)
    mu_Q = np.sum(Q*W[:,None], axis=0)/np.sum(W)
    X = (P-mu_P).T @ W_mat @ (Q-mu_Q)
    U,S,Vt = svd(X)
    R = Vt.T@U.T
    if np.linalg.det(R)<0:
        Vt[-1,:]*=-1; R = Vt.T@U.T
    t = mu_P - R@mu_Q
    return R, t

def data_cb(robot, msg):
    global other_data
    arr = np.array(msg.data)
    n = int(len(arr)/3)
    pts = arr.reshape(n,3)
    other_data[robot] = pts

def process():
    if robot_pose is None: return
    X_meas.append(robot_pose.copy())
    for i in range(num_aps):
        Y_meas[i].append(simulate_rssi(robot_pose, AP_true[i]))
    if len(X_meas)<10: return
    models, means, stds = gp_model()
    ap_globals = []; ap_candidates = []; weights_all = []
    for i in range(num_aps):
        glob, cands = detect_maxima(means[i])
        ap_globals.append(glob)
        cand_ws = []
        for cand in cands:
            idx0 = (np.abs(xx - cand[0])).argmin()
            idx1 = (np.abs(yy - cand[1])).argmin()
            w = weight_candidate(stds[i][idx1, idx0])
            cand_ws.append(w)
        ap_candidates.append(cands)
        weights_all.append(cand_ws)
    pts_for_hull = []
    for i in range(num_aps):
        pts = [ap_globals[i]]
        pts.extend(ap_candidates[i])
        pts_for_hull.extend(pts)
    pts_for_hull = np.array(pts_for_hull)
    if pts_for_hull.shape[0]>=3:
        hull = ConvexHull(pts_for_hull)
        hull_pts = pts_for_hull[hull.vertices]
    else:
        hull_pts = pts_for_hull
    msg_data = []
    for i in range(num_aps):
        msg_data.extend(ap_globals[i].tolist())
    for pt in hull_pts:
        msg_data.extend(pt.tolist())
    for i in range(num_aps):
        if len(weights_all[i])>0:
            msg_data.append(np.mean(weights_all[i]))
        else:
            msg_data.append(0.0)
    msg_pub = Float32MultiArray()
    msg_pub.data = msg_data
    pub.publish(msg_pub)
    rel_arr = []
    for other in other_data.values():
        if other.shape[0]>=3 and hull_pts.shape[0]>=3:
            common = np.minimum(hull_pts.shape[0], other.shape[0])
            R,t = svd_alignment(hull_pts[:common], other[:common,:2], [1.0]*common)
            rel_arr.extend(t.tolist())
        else:
            rel_arr.extend([0.0,0.0])
    rel_msg = Float32MultiArray()
    rel_msg.data = rel_arr
    pose_pub.publish(rel_msg)
while not rospy.is_shutdown():
    process()
    rate.sleep()
rospy.spin()
