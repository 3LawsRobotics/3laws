/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_KERNEL_GENERATOR_NODE_UNICYCLE_HPP
#define THREELAWS_KERNEL_GENERATOR_NODE_UNICYCLE_HPP

#include "./kernel_generator_node.hpp"

namespace lll {

struct UnicycleDynamicsParams
{
  double Cv = 1.;
  double Cw = 1.;
};

struct UnicycleMapParams
{
  std::size_t n_obstacles = 100;
};

struct UnicycleFailsafeParams
{
  double Kv    = 5.0;
  double Kw    = 1.0;
  double max_v = 2.0;
  double R     = 1.0;
};

struct KernelGeneratorNodeUnicycleParams
{
  KernelGeneratorNodeParams kernel_generator;
  UnicycleDynamicsParams dynamics;
  UnicycleMapParams map;
  UnicycleFailsafeParams failsafe;
};

// Component
class KernelGeneratorNodeUnicycle : public KernelGeneratorNode
{
public:
  explicit KernelGeneratorNodeUnicycle(const rclcpp::NodeOptions & options);

  void init();

private:
  KernelGeneratorNodeUnicycleParams m_prm;
};

}  // namespace lll

BOOST_HANA_ADAPT_STRUCT(lll::UnicycleDynamicsParams, Cv, Cw);
BOOST_HANA_ADAPT_STRUCT(lll::UnicycleMapParams, n_obstacles);
BOOST_HANA_ADAPT_STRUCT(lll::UnicycleFailsafeParams, Kv, Kw, max_v, R);
BOOST_HANA_ADAPT_STRUCT(
  lll::KernelGeneratorNodeUnicycleParams, kernel_generator, dynamics, map, failsafe);

// def get_lidar_data(self,step_size):
//     global lidar_sensor_interface, min_distance, min_angle
//     data =
//     lidar_sensor_interface.get_linear_depth_data("/World/Carter/chassis_link/carter_lidar")
//     min_distance = data.min()
//     min_idx = data.argmin()
//     azi = lidar_sensor_interface.get_azimuth_data("/World/Carter/chassis_link/carter_lidar")
//     pcd = lidar_sensor_interface.get_point_cloud_data("/World/Carter/chassis_link/carter_lidar")
//     min_angle = azi[min_idx]+math.pi/2
//     # print("d: " + str(min_distance) + "azi: " + str(min_angle))
//     self.min_distance = min_distance
//     self.min_angle = min_angle
//     position, orientation = self._carter.get_world_pose()
//     theta = omni.isaac.core.utils.rotations.quat_to_euler_angles(orientation)[2]
//     x = position[0]
//     y = position[1]
//     point = pcd[min_idx]
//     T =
//     np.array([[math.cos(theta),-math.sin(theta),x],[math.sin(theta),math.cos(theta),y],[0,0,1]])
//     P = np.array([-point[0,1],point[0,0],1])
//     # tmp = np.matmul(np.linalg.inv(T),np.transpose(P))
//     tmp = np.matmul(T,np.transpose(P))
//     self.obs_p = np.array([tmp[0],tmp[1]])
//     return

// def backup_safety_set(self):
//     T = 3
//     dt = 0.01
//     t = 0
//     self.h = np.linalg.norm(np.array([self.obs_p[0]-self.x[0],self.obs_p[1]-self.x[1]]))
//     while t < T:
//         self.backup_controller()
//         self.u = self.u_b
//         self.full_dynamics()
//         self.x = self.x + self.xDot * dt
//         h = np.linalg.norm(np.array([self.obs_p[0]-self.x[0],self.obs_p[1]-self.x[1]]))
//         if (self.h > h):
//             self.h = h
//         t = t+dt
//     return

// def barrier(self):
//     self.r = 0.5
//     if (move_forklifts):
//         self.r = 1.0
//     beta = 1
//     self.backup_safety_set()
//     vperp = self.x[2]*math.cos(self.min_angle)
//     if (vperp < .1):
//         vperp = .1
//     if (vperp < .1 and move_forklifts):
//         vperp = .5

//     d = self.min_distance-self.r
//     if (d < 0):
//         d = 0
//     lambda1 = 1-math.exp(-beta*d/vperp)
//     if (self.h < 4*self.r):
//         self.backup_controller()
//         self.u_act = lambda1*self.u_des+(1-lambda1)*self.u_b
//         # print("backup controller engaged")
//     else:
//         self.u_act = self.u_des
//     if (supervisor==False):
//         self.u_act = self.u_des
//     return

// def backup_controller(self):
//     Kv = 5
//     Kw = 1
//     self.u_b = np.array([0.0,0.0])
//     direction = 1
//     max_v = 2
//     if (self.min_distance < 2*self.r): # move away from obstacle
//         if (math.fabs(self.min_angle) < math.pi/2):
//             direction = -1
//         else:
//             direction = 1
//         self.u_b[0] = direction*Kv*math.fabs(self.min_distance-2*self.r)
//         if (direction == -1):
//             self.u_b[1] = Kw*(self.min_angle)
//         else:
//             tmp = self.min_angle % math.copysign(1,self.min_angle)*math.pi
//             if (tmp > 0):
//                 tmp = tmp-math.pi
//             else:
//                 tmp = tmp+math.pi
//             self.u_b[1] = Kw*tmp
//     if (self.u_b[0] > max_v):
//         self.u_b[0] = max_v
//     if (self.u_b[0] < -max_v):
//         self.u_b[0] = -max_v

//     return

#endif  // THREELAWS_KERNEL_GENERATOR_NODE_UNICYCLE_HPP
