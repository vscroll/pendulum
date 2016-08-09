#ifndef __PENDULUM_DYNAMIC_H__
#define __PENDULUM_DYNAMIC_H__

#include <vector>
#include <geometry_msgs/Vector3.h>
#include <math.h>

/*
 ==============================================

 <A Flying Inverted Pendulum.pdf>
 ==============================================
 */


/*

    |1 0            0            |
RMx=|0 cos(angle_x) -sin(angle_x)|
    |0 sin(angle_x) cos(angle_x) |

    |cos(angle_y)  0 sin(angle_y)|
RMy=|0             1 0           |
    |-sin(angle_y) 0 cos(angle_y)|

    |cos(angle_z) -sin(angle_z) 0|
RMz=|sin(angle_z) cos(angle_z)  0|
    |0            0             1|

double g = 9.81;

*/

class PendulumDynamic {

public:

	static constexpr double g = 9.80;

	/*
	formula 12:

	syms r r_vel r_acc s s_vel s_acc x_acc y_acc L g;
	l=sqrt(L^2-r^2-s^2)
	z_acc=0

	f1 =
	r_acc == -(x_acc*(L^2 - s^2)^2 - 2*r^2*(x_acc*(L^2 - s^2) - r_vel*s*s_vel) + r^4*x_acc + r*(L^2*(r_vel^2 + s_vel^2 - l*(g + z_acc)) - s^3*s_acc + s^2*(- r_vel^2 + l*(g + z_acc)) + L^2*s*s_acc) - r^3*(s_vel^2 - l*(g + z_acc) + s*s_acc))/(l^2*(L^2 - s^2))

	f2 =
	s_acc == -(y_acc*(L^2 - r^2)^2 - 2*s^2*(y_acc*(L^2 - r^2) - r*r_vel*s_vel) + s^4*y_acc + s*(L^2*(r_vel^2 + s_vel^2 - l*(g + z_acc)) - r^3*r_acc + r^2*(- s_vel^2 + l*(g + z_acc)) + L^2*r*r_acc) - s^3*(r_vel^2 - l*(g + z_acc) + r*r_acc))/(l^2*(L^2 - s^2))

	*/
	static bool formula_12(const double pendulum_l,
						const double pendulum_s,
						const double pendulum_r,
						const geometry_msgs::Vector3& pendulum_vel,
						const geometry_msgs::Vector3& pendulum_vel_acc,
						double* vehicle_vel_acc_x,
						double* vehicle_vel_acc_y) {

		double l = sqrt(pow(pendulum_l, 2) - pow(pendulum_r, 2) - pow(pendulum_s, 2));
		double z_acc = 0;

		double l_2 = pow(l, 2);
		double L_2 = pow(pendulum_l, 2);

		double r = pendulum_r;
		double r_2 = pow(r, 2);
		double r_3 = pow(r, 3);
		double r_4 = pow(r, 4);
		double r_vel = pendulum_vel.x;
		double r_vel_2 = pow(pendulum_vel.x, 2);
		double r_acc = pendulum_vel_acc.x;

		double s = pendulum_s;
		double s_2 = pow(s, 2);
		double s_3 = pow(s, 3);
		double s_4 = pow(s, 4);
		double s_vel = pendulum_vel.y;
		double s_vel_2 = pow(s_vel, 2);
		double s_acc = pendulum_vel_acc.y;

		/*
		   ans =

		   -(l^2*(L^2 - s^2)*(r_acc + ((- s_vel^2 + l*(g + z_acc) - s*s_acc)*r^3 + 
		   2*r_vel*s*s_vel*r^2 + (s_acc*L^2*s + (r_vel^2 + s_vel^2 - l*(g + z_acc))*L^2 - 
		   s_acc*s^3 + (l*(g + z_acc) - r_vel^2)*s^2)*r)/(l^2*(L^2 - s^2))))/((L^2 - s^2)^2 -
		   2*r^2*(L^2 - s^2) + r^4)
		 */
		*vehicle_vel_acc_x =
			-(l_2*(L_2 - s_2)*(r_acc +	((-s_vel_2 + l*(g + z_acc) - s*s_acc)*r_3 +
			2*r_vel*s*s_vel*r_2 + (s_acc*L_2*s + (r_vel_2 + s_vel_2 - l*(g + z_acc))*L_2 -
			s_acc*s_3 + (l*(g + z_acc) - r_vel_2)*s_2)*r)/(l_2*(L_2 - s_2))))/(pow((L_2 - s_2), 2) -
			2*r_2*(L_2 - s_2) + r_4);

		/*
		   ans =

		   -(l^2*(L^2 - s^2)*(s_acc + ((- r_vel^2 + l*(g + z_acc) - r*r_acc)*s^3 + 
		   2*r*r_vel*s_vel*s^2 + (r_acc*L^2*r + (r_vel^2 + s_vel^2 - l*(g + z_acc))*L^2 - 
		   r_acc*r^3 + (l*(g + z_acc) - s_vel^2)*r^2)*s)/(l^2*(L^2 - s^2))))/((L^2 - r^2)^2 -
		   2*s^2*(L^2 - r^2) + s^4)
		 */
		*vehicle_vel_acc_y =
			-(l_2*(L_2 - s_2)*(s_acc +	((-r_vel_2 + l*(g + z_acc) - r*r_acc)*s_3 +
			2*r*r_vel*s_vel*s_2 + (r_acc*L_2*r + (r_vel_2 + s_vel_2 - l*(g + z_acc))*L_2 -
			r_acc*r_3 + (l*(g + z_acc) - s_vel_2)*r_2)*s)/(l_2*(L_2 - s_2))))/(pow((L_2 - r_2), 2) -
			2*s_2*(L_2 - r_2) + s_4);

		return isnan(*vehicle_vel_acc_x) || isnan(*vehicle_vel_acc_y);
	}

	/*
	version 1:

	|vel_acc_x|             |0|   |0|
	|vel_acc_y|=RMx*RMy*RMz*|0| + |0|
	|vel_acc_z|             |a|   |g|
=>
	|vel_acc_x|  |a*sin(angle_y)                 |
	|vel_acc_y|= |-a*cos(angle_y)*sin(angle_x)   |
	|vel_acc_z|  |a*cos(angle_x)*cos(angle_y) - g|
=>
	vel_acc_x=a*sin(angle_y)
	vel_acc_y=-a*cos(angle_y)*sin(angle_x)
	vel_acc_z=a*cos(angle_y)*cos(angle_x)-g
	vel_acc_z=0

	*/
	static void formula_5(const double vel_acc_x, const double vel_acc_y,
						double* angle_x, double* angle_y, double* a) {

		*angle_x = atan(-vel_acc_y/g);
		*angle_y = atan(-vel_acc_x*sin(*angle_x)/vel_acc_y);
		*a = vel_acc_x/sin(*angle_y);
	}

	/*
	version 2:

	|vel_acc_x|             |0|   |0|
	|vel_acc_y|=RMz*RMy*RMx*|0| + |0|
	|vel_acc_z|             |a|   |g|
=>
	|vel_acc_x| |cos(angle_z) -sin(angle_z) 0|   |cos(angle_y)  0 sin(angle_y)|   |1 0            0            |   |0|   |0|
	|vel_acc_y|=|sin(angle_z) cos(angle_z)  0| * |0             1 0           | * |0 cos(angle_x) -sin(angle_x)| * |0| + |0|
	|vel_acc_z| |0            0             1|   |-sin(angle_y) 0 cos(angle_y)|   |0 sin(angle_x) cos(angle_x) |   |a|   |g|

	            |a*(sin(angle_x)*sin(angle_z) + cos(angle_x)*cos(angle_z)*sin(angle_y)) |
	           =|-a*(cos(angle_z)*sin(angle_x) - cos(angle_x)*sin(angle_y)*sin(angle_z))|
	            |a*cos(angle_x)*cos(angle_y) - g                                        |

	vel_acc_z=0
	angle_z=0
	cos(angle_z)=1
	sin(angle_z)=0
=>
	vel_acc_x=a*cos(angle_x)*sin(angle_y)
	vel_acc_y=-a*sin(angle_x)
	0=a*cos(angle_x)cos(angle_y)-g

	*/
	static void formula_5_2(const double vel_acc_x, const double vel_acc_y,
						double* angle_x, double* angle_y, double* a) {


		*angle_y = atan(vel_acc_x/g);
		*angle_x = atan(-vel_acc_y*sin(*angle_y)/vel_acc_x);
		//*a = -vel_acc_y/sin(*angle_x);
		*a = g/(cos(*angle_x)*cos(*angle_y));
	}

	/*
	version 1:

	| rate_x |   | cos(angle_y)*cos(angle_x)	-sin(angle_x)	0 |-1    | vehicle_rate_x |
	| rate_y | = | cos(angle_y)*sin(angle_x)	cos(angle_x)	0 |   *  | vehicle_rate_y |
	| rate_z |   | -sin(angle_y)				0				1 |      | vehicle_rate_z |
=>
	| rate_x*cos(angle_x)*cos(angle_y) - rate_y*sin(angle_x) |   | vehicle_rate_x |
	| rate_y*cos(angle_x) + rate_x*cos(angle_y)*sin(angle_x) | = | vehicle_rate_y |
	| rate_z - rate_x*sin(angle_y)                           |   | vehicle_rate_z |

	*/
	static void formula_7(const double angle_x, const double angle_y,
						const double rate_x, const double rate_y,
						double* vehicle_rate_x, double* vehicle_rate_y) {


		*vehicle_rate_x = rate_x*cos(angle_x)*cos(angle_y) - rate_y*sin(angle_x);
		*vehicle_rate_y = rate_y*cos(angle_x) + rate_x*cos(angle_y)*sin(angle_x);
	}

	/*
	version 2:

	|vehicle_rate_x|   |rate_x|            |0     |                     |0     |
	|vehicle_rate_y| = |0     | + RMx^-1 * |rate_y| + RMx^-1 * RMy^-1 * |0     |
	|vehicle_rate_z|   |0     |            |0     |                     |rate_z|
=>
	|vehicle_rate_x|   |rate_x|   |1 0             0           |   |0     |
	|vehicle_rate_y| = |0     | + |0 cos(angle_x)  sin(angle_x)| * |rate_y|
	|vehicle_rate_z|   |0     |   |0 -sin(angle_x) cos(angle_x)|   |0     |

	                              |1 0             0           |   |cos(angle_y) 0 -sin(angle_y)|   |0     |
	                            + |0 cos(angle_x)  sin(angle_x)| * |0            1 0            | * |0     |
	                              |0 -sin(angle_x) cos(angle_x)|   |sin(angle_y) 0 cos(angle_y) |   |rate_z|
=>
	|vehicle_rate_x|   |rate_x + 0 - sin(angle_y)*rate_z                          |
	|vehicle_rate_y| = |0 + cos(angle_x)*rate_y + sin(angle_x)*cos(angle_y)*rate_z|
	|vehicle_rate_z|   |0 - sin(angle_x)*rate_y + cos(angle_x)*cos(angle_y)*rate_z|
=>
	|vehicle_rate_x|   |1 0             -sin(angle_y)            |   |rate_x|
	|vehicle_rate_y| = |0 cos(angle_x)  sin(angle_x)*cos(angle_y)| * |rate_y|
	|vehicle_rate_z|   |0 -sin(angle_x) cos(angle_x)*cos(angle_y)|   |rate_z|

	*/
	static void formula_7_2(const double angle_x, const double angle_y,
						const double rate_x, const double rate_y,
						double* vehicle_rate_x, double* vehicle_rate_y) {

		double rate_z = 0;
		*vehicle_rate_x = rate_x - sin(angle_y)*rate_z;
		*vehicle_rate_y = cos(angle_x)*rate_y + sin(angle_x)*cos(angle_y)*rate_z;
	}
};

/*
 ==============================================

 <Cascaded control for balancing an inverted pendulum on a flying quadrotor.pdf>
 ================================================
*/

class PendulumDynamic2 {

public:
	static constexpr double g = 9.80;

	/*
	formula 4, 5:

	*/
	static bool formula_4_5(const double pendulum_l,
						const double pendulum_r,
						const double pendulum_s,
						const geometry_msgs::Vector3& pendulum_vel,
						const geometry_msgs::Vector3& pendulum_vel_acc,
						double* vehicle_vel_acc_x,
						double* vehicle_vel_acc_y) {

		double l = sqrt(pow(pendulum_l, 2) - pow(pendulum_r, 2) - pow(pendulum_s, 2));
		double z_acc = 0;

		double l_2 = pow(l, 2);
		double L_2 = pow(pendulum_l, 2);

		double r = pendulum_r;
		double r_2 = pow(r, 2);
		double r_3 = pow(r, 3);
		double r_vel = pendulum_vel.x;
		double r_vel_2 = pow(pendulum_vel.x, 2);
		double r_acc = pendulum_vel_acc.x;

		double s = pendulum_s;
		double s_2 = pow(s, 2);
		double s_3 = pow(s, 3);
		double s_vel = pendulum_vel.y;
		double s_vel_2 = pow(s_vel, 2);
		double s_acc = pendulum_vel_acc.y;

		*vehicle_vel_acc_x =
			(3*r*l*(g+z_acc) + 4*(r_3*(s_vel_2 + s*s_acc) - 2*r_2*s*r_vel*s_vel +
			r*(-L_2*s*s_acc + s_3*s_acc + s_2*r_vel_2 - L_2*r_vel_2 - L_2*s_vel_2))/l_2 -
			4*(L_2 - s_2)*r_acc)/(3*l_2);

		*vehicle_vel_acc_y =
			(3*s*l*(g+z_acc) + 4*(s_3*(r_vel_2 + r*r_acc) - 2*s_2*r*s_vel*r_vel +
			s*(-L_2*r*r_acc + r_3*r_acc + r_2*s_vel_2 - L_2*s_vel_2 - L_2*r_vel_2))/l_2 -
			4*(L_2 - r_2)*s_acc)/(3*l_2);

		return isnan(*vehicle_vel_acc_x) || isnan(*vehicle_vel_acc_y);
	}

	/*
	formula 4, 5, 12:

	*/
	static bool formula_4_5_12(const double pendulum_l,
						const double pendulum_r,
						const double pendulum_s,
						const geometry_msgs::Vector3& pendulum_vel,
						const geometry_msgs::Vector3& pendulum_vel_acc,
						double* vehicle_angular_x,
						double* vehicle_angular_y) {

		double l = sqrt(pow(pendulum_l, 2) - pow(pendulum_r, 2) - pow(pendulum_s, 2));
		double z_acc = 0;

		double l_2 = pow(l, 2);
		double L_2 = pow(pendulum_l, 2);

		double r = pendulum_r;
		double r_2 = pow(r, 2);
		double r_3 = pow(r, 3);
		double r_vel = pendulum_vel.x;
		double r_vel_2 = pow(pendulum_vel.x, 2);
		double r_acc = pendulum_vel_acc.x;

		double s = pendulum_s;
		double s_2 = pow(s, 2);
		double s_3 = pow(s, 3);
		double s_vel = pendulum_vel.y;
		double s_vel_2 = pow(s_vel, 2);
		double s_acc = pendulum_vel_acc.y;

		*vehicle_angular_y =
			atan(((3*r*l*(g+z_acc) + 4*(r_3*(s_vel_2 + s*s_acc) - 2*r_2*s*r_vel*s_vel +
			r*(-L_2*s*s_acc + s_3*s_acc + s_2*r_vel_2 - L_2*r_vel_2 - L_2*s_vel_2))/l_2 -
			4*(L_2 - s_2)*r_acc)/(3*l_2))/g);

		*vehicle_angular_x =
			atan(((3*s*l*(g+z_acc) + 4*(s_3*(r_vel_2 + r*r_acc) - 2*s_2*r*s_vel*r_vel +
			s*(-L_2*r*r_acc + r_3*r_acc + r_2*s_vel_2 - L_2*s_vel_2 - L_2*r_vel_2))/l_2 -
			4*(L_2 - r_2)*s_acc)/(3*l_2))*(-cos(*vehicle_angular_y ))/g);

		return isnan(*vehicle_angular_x) || isnan(*vehicle_angular_y);
	}
};

#endif
