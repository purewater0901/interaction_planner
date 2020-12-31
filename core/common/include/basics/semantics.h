#ifndef SRC_SEMANTICS_H
#define SRC_SEMANTICS_H

#include <cassert>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "basics/basics.h"
#include "basics/shapes.h"
#include "basics/tool_func.h"
#include "lane/lane.h"
#include "state/free_state.h"
#include "state/frenet_state.h"
#include "state/state.h"

namespace common
{
    class VehicleParam
    {
    public:
        inline double width() const { return width_; }
        inline double length() const { return length_; }
        inline double wheel_base() const { return wheel_base_; }
        inline double front_suspension() const { return front_suspension_; }
        inline double rear_suspension() const { return rear_suspension_; }
        inline double max_steering_angle() const { return max_steering_angle_; }
        inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
        inline double max_lateral_acc() const { return max_lateral_acc_; }
        inline double d_cr() const { return d_cr_; }

        inline void set_width(const double val) { width_ = val; }
        inline void set_length(const double val) { length_ = val; }
        inline void set_wheel_base(const double val) { wheel_base_ = val; }
        inline void set_front_suspension(const double val) {
            front_suspension_ = val;
        }
        inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
        inline void set_max_steering_angle(const double val) {
            max_steering_angle_ = val;
        }
        inline void set_max_longitudinal_acc(const double val) {
            max_longitudinal_acc_ = val;
        }
        inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
        inline void set_d_cr(const double val) { d_cr_ = val; }

        /**
         * @brief Print info
         */
        void print() const;

    private:
        double width_ = 1.90;
        double length_ = 4.88;
        double wheel_base_ = 2.85;
        double front_suspension_ = 0.93;
        double rear_suspension_ = 1.10;
        double max_steering_angle_ = 45.0;

        double max_longitudinal_acc_ = 2.0;
        double max_lateral_acc_ = 2.0;

        double d_cr_ = 1.34;  // length between geometry center and rear axle
    };

    class Vehicle {
    public:
        Vehicle();
        Vehicle(const VehicleParam &param, const State &state);
        Vehicle(const int &id, const VehicleParam &param, const State &state);
        Vehicle(const int &id, const std::string &subclass, const VehicleParam &param,
                const State &state);

        inline int id() const { return id_; }
        inline std::string subclass() const { return subclass_; }
        inline VehicleParam param() const { return param_; }
        inline State state() const { return state_; }
        inline std::string type() const { return type_; }

        inline void set_id(const int &id) { id_ = id; }
        inline void set_subclass(const std::string &subclass) {
            subclass_ = subclass;
        }
        inline void set_type(const std::string &type) { type_ = type; }
        inline void set_param(const VehicleParam &in) { param_ = in; }
        inline void set_state(const State &in) { state_ = in; }

        /**
         * @brief Get 3-DoF vehicle state at center of rear axle, x-y-yaw
         *
         * @return Eigen::Matrix<double,3,1> 3-dof vehicle state
         */
        Eigen::Matrix<double,3,1> Ret3DofState() const;

        /**
         * @brief Get 2D OBB of the vehicle
         *
         * @return OrientedBoundingBox2D OBB
         */
        OrientedBoundingBox2D RetOrientedBoundingBox() const;

        /**
         * @brief Return color in jet colormap using mapping function
         *
         * @param vertices pointer of vertice container
         * @return ErrorType
         */
        ErrorType RetVehicleVertices(vec_E<Eigen::Matrix<double,2,1>> *vertices) const;

        /**
         * @brief Return front and rear points on longitudinal axle
         *
         * @param vertices
         * @return ErrorType
         */
        ErrorType RetBumperVertices(std::array<Eigen::Matrix<double,2,1>, 2> *vertices) const;

        /**
         * @brief Return 3-DoF state at geometry center, x-y-yaw
         *
         * @param state pointer of state
         * @return ErrorType
         */
        ErrorType Ret3DofStateAtGeometryCenter(Eigen::Matrix<double,3,1> *state) const;

        /**
         * @brief Print info
         */
        void print() const;

    private:
        int id_{constants::kInvalidAgentId};
        std::string subclass_;
        std::string type_;
        VehicleParam param_;
        State state_;
    };

    enum class LongitudinalBehavior {
        kMaintain = 0,
        kAccelerate,
        kDecelerate,
        kStopping
    };

    enum class LateralBehavior {
        kUndefined = 0,
        kLaneKeeping,
        kLaneChangeLeft,
        kLaneChangeRight,
    };

    struct EnumClassHash {
        template <typename T>
        std::size_t operator()(T t) const {
            return static_cast<std::size_t>(t);
        }
    };

    struct ProbDistOfLatBehaviors {
        bool is_valid = false;
        std::unordered_map<LateralBehavior, double, EnumClassHash> probs{
                {common::LateralBehavior::kLaneChangeLeft, 0.0},
                {common::LateralBehavior::kLaneChangeRight, 0.0},
                {common::LateralBehavior::kLaneKeeping, 0.0}};

        void SetEntry(const LateralBehavior &beh, const double &val) {
            probs[beh] = val;
        }

        bool CheckIfNormalized() const {
            double sum = 0.0;
            for (const auto &entry : probs) {
                sum += entry.second;
            }
            if (fabs(sum - 1.0) < constants::kEPS) {
                return true;
            } else {
                return false;
            }
        }

        bool GetMaxProbBehavior(LateralBehavior *beh) const {
            if (!is_valid) return false;

            double max_prob = -1.0;
            LateralBehavior max_beh;
            for (const auto &entry : probs) {
                if (entry.second > max_prob) {
                    max_prob = entry.second;
                    max_beh = entry.first;
                }
            }
            *beh = max_beh;
            return true;
        }
    };

/**
 * @brief Semantic behavior is a collection of different behaviors
 * to describe a complex intention.
 * @param ref_lane, may not be a lane existed in the physical
 * world, instead, it may be reconstructed using the physical
 * lanes as well as the discret behavior etc.
 */

    struct SemanticBehavior {
        LateralBehavior lat_behavior;
        LongitudinalBehavior lon_behavior;
        Lane ref_lane;
        double actual_desired_velocity{0.0};

        vec_E<vec_E<Vehicle>> forward_trajs;
        std::vector<LateralBehavior> forward_behaviors;
        vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs;

        State state;

        SemanticBehavior() {
            lat_behavior = LateralBehavior::kLaneKeeping;
            lon_behavior = LongitudinalBehavior::kMaintain;
        }
        SemanticBehavior(const LateralBehavior &beh) : lat_behavior(beh) {}
    };

/**
 * @brief Vehicle with semantic info
 */
    struct SemanticVehicle {
        // * vehicle
        Vehicle vehicle;

        // * nearest lane info
        int nearest_lane_id{constants::kInvalidLaneId};
        double dist_to_lane{-1.0};
        double arc_len_onlane{-1.0};

        // * prediction
        ProbDistOfLatBehaviors probs_lat_behaviors;

        // * argmax behavior
        LateralBehavior lat_behavior{LateralBehavior::kUndefined};
        Lane lane;
    };

    struct SemanticVehicleSet {
        std::unordered_map<int, SemanticVehicle> semantic_vehicles;
    };

    struct VehicleSet {
        std::unordered_map<int, Vehicle> vehicles;

        /**
         * @brief Print info
         */
        void print() const;
    };

/**
 * @brief Vehicle info under Frenet-frame
 */
    struct FsVehicle {
        FrenetState frenet_state;
        vec_E<Eigen::Matrix<double,2,1>> vertices;
    };

/**
 * @brief Vehicle control signal, 2 modes embedded
 * @brief open-loop: use desired state
 * @brief closed-loop: use longitudinal acc and steering rate
 */
    struct VehicleControlSignal {
        double acc = 0.0;
        double steer_rate = 0.0;
        bool is_openloop = false;
        common::State state;

        /**
         * @brief Default constructor
         */
        VehicleControlSignal();

        /**
         * @brief Construct a new Vehicle Control Signal object
         *
         * @param acc longitudinal acc, m/s^2
         * @param steer_rate steering rate, rad/s
         */
        VehicleControlSignal(double acc, double steer_rate);

        /**
         * @brief Construct a new Vehicle Control Signal object
         *
         * @param state desired state
         */
        VehicleControlSignal(common::State state);
    };

    struct VehicleControlSignalSet {
        std::unordered_map<int, VehicleControlSignal> signal_set;
    };

    struct GridMapMetaInfo {
        int width = 0;
        int height = 0;
        double resolution = 0;
        double w_metric = 0;
        double h_metric = 0;

        /**
         * @brief Construct a new Grid Map Meta Info object
         */
        GridMapMetaInfo();

        /**
         * @brief Construct a new Grid Map Meta Info object
         *
         * @param w width
         * @param h height
         * @param res resolution
         */
        GridMapMetaInfo(const int w, const int h, const double res);

        /**
         * @brief Print info
         */
        void print() const;
    };

/**
 * @brief Occupancy grid map
 *
 * @tparam T Data type of the grid map
 * @tparam N_DIM Dimension of the grid map
 */
    template <typename T, int N_DIM>
    class GridMapND {
    public:
        enum ValType {
            OCCUPIED = 70,
            // FREE = 102,
            FREE = 0,
            SCANNED_OCCUPIED = 128,
            UNKNOWN = 0
        };

        /**
         * @brief Construct a new GridMapND object
         *
         */
        GridMapND();

        /**
         * @brief Construct a new GridMapND object
         *
         * @param dims_size size of each dimension
         * @param dims_resolution resolution of each dimension
         * @param dims_name name of each dimension
         */
        GridMapND(const std::array<int, N_DIM> &dims_size,
                  const std::array<double, N_DIM> &dims_resolution,
                  const std::array<std::string, N_DIM> &dims_name);

        inline std::array<int, N_DIM> dims_size() const { return dims_size_; }
        inline int dims_size(const int &dim) const { return dims_size_.at(dim); }
        inline std::array<int, N_DIM> dims_step() const { return dims_step_; }
        inline int dims_step(const int &dim) const { return dims_step_.at(dim); }
        inline std::array<double, N_DIM> dims_resolution() const {
            return dims_resolution_;
        }
        inline double dims_resolution(const int &dim) const {
            return dims_resolution_.at(dim);
        }
        inline std::array<std::string, N_DIM> dims_name() const { return dims_name_; }
        inline std::string dims_name(const int &dim) const {
            return dims_name_.at(dim);
        }
        inline std::array<double, N_DIM> origin() const { return origin_; }
        inline int data_size() const { return data_size_; }
        inline const std::vector<T> *data() const { return &data_; }
        inline T data(const int &i) const { return data_[i]; };
        inline T *get_data_ptr() { return data_.data(); }
        inline const T *data_ptr() const { return data_.data(); }

        inline void set_origin(const std::array<double, N_DIM> &origin) {
            origin_ = origin;
        }
        inline void set_dims_size(const std::array<int, N_DIM> &dims_size) {
            dims_size_ = dims_size;
            SetNDimSteps(dims_size);
            SetDataSize(dims_size);
        }
        inline void set_dims_resolution(
                const std::array<double, N_DIM> &dims_resolution) {
            dims_resolution_ = dims_resolution;
        }
        inline void set_dims_name(const std::array<std::string, N_DIM> &dims_name) {
            dims_name_ = dims_name;
        }
        inline void set_data(const std::vector<T> &in) { data_ = in; }

        /**
         * @brief Set all data in array to 0
         */
        inline void clear_data() { data_ = std::vector<T>(data_size_, 0); }

        /**
         * @brief Fill all data in array using value
         *
         * @param val input value
         */
        inline void fill_data(const T &val) {
            data_ = std::vector<T>(data_size_, val);
        }

        /**
         * @brief Get the Value Using Coordinate
         *
         * @param coord Input coordinate
         * @param val Output value
         * @return ErrorType
         */
        ErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                          T *val) const;

        /**
         * @brief Get the Value Using Global Position
         *
         * @param p_w Input global position
         * @param val Output value
         * @return ErrorType
         */
        ErrorType GetValueUsingGlobalPosition(const std::array<double, N_DIM> &p_w,
                                              T *val) const;

        /**
         * @brief Check if the input value is equal to the value in map
         *
         * @param p_w Input global position
         * @param val_in Input value
         * @param res Result
         * @return ErrorType
         */
        ErrorType CheckIfEqualUsingGlobalPosition(
                const std::array<double, N_DIM> &p_w, const T &val_in,
                bool *res) const;

        /**
         * @brief Check if the input value is equal to the value in map
         *
         * @param coord Input coordinate
         * @param val_in Input value
         * @param res Result
         * @return ErrorType
         */
        ErrorType CheckIfEqualUsingCoordinate(const std::array<int, N_DIM> &coord,
                                              const T &val_in, bool *res) const;

        /**
         * @brief Set the Value Using Coordinate
         *
         * @param coord Coordinate of the map
         * @param val Input value
         * @return ErrorType
         */
        ErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                          const T &val);

        /**
         * @brief Set the Value Using Global Position
         *
         * @param p_w Global position
         * @param val Input value
         * @return ErrorType
         */
        ErrorType SetValueUsingGlobalPosition(const std::array<double, N_DIM> &p_w,
                                              const T &val);

        /**
         * @brief Get the Coordinate Using Global Position
         *
         * @param p_w Input global position
         * @return std::array<int, N_DIM> Output coordinate
         */
        std::array<int, N_DIM> GetCoordUsingGlobalPosition(
                const std::array<double, N_DIM> &p_w) const;

        /**
         * @brief Get the Rounded Position Using Global Position object
         *
         * @param p_w Input global position
         * @return std::array<double, N_DIM> Output global position
         */
        std::array<double, N_DIM> GetRoundedPosUsingGlobalPosition(
                const std::array<double, N_DIM> &p_w) const;

        /**
         * @brief Get the Global Position Using Coordinate
         *
         * @param coord Input coordinate
         * @param p_w Output global position
         * @return ErrorType
         */
        ErrorType GetGlobalPositionUsingCoordinate(
                const std::array<int, N_DIM> &coord,
                std::array<double, N_DIM> *p_w) const;

        /**
         * @brief Get the Coordinate Using Global Metric On Single Dimension
         *
         * @param metric Input global 1-dim position
         * @param i Dimension
         * @param idx Output 1-d coordinate
         * @return ErrorType
         */
        ErrorType GetCoordUsingGlobalMetricOnSingleDim(const double &metric,
                                                       const int &i, int *idx) const;

        /**
         * @brief Get the Global Metric Using Coordinate On Single Dim object
         *
         * @param idx Input 1-d coordinate
         * @param i Dimension
         * @param metric Output 1-d position
         * @return ErrorType
         */
        ErrorType GetGlobalMetricUsingCoordOnSingleDim(const int &idx, const int &i,
                                                       double *metric) const;

        /**
         * @brief Check if the input coordinate is in map range
         *
         * @param coord Input coordinate
         * @return true In range
         * @return false Out of range
         */
        bool CheckCoordInRange(const std::array<int, N_DIM> &coord) const;

        /**
         * @brief Check if the input 1-d coordinate is in map range
         *
         * @param idx Input 1-d coordinate
         * @param i Dimension
         * @return true In range
         * @return false Out of range
         */
        bool CheckCoordInRangeOnSingleDim(const int &idx, const int &i) const;

        /**
         * @brief Get the mono index using N-dim index
         *
         * @param idx Input N-dim index
         * @return int Output 1-dim index
         */
        int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> &idx) const;

        /**
         * @brief Get N-dim index using mono index
         *
         * @param idx Input mono index
         * @return std::array<int, N_DIM> Output N-dim index
         */
        std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int &idx) const;

    private:
        /**
         * @brief Set the steps of N-dim array
         * @brief E.g. A x-y-z map's steps are {1, x, x*y}
         *
         * @param dims_size Input the size of dimension
         * @return ErrorType
         */
        ErrorType SetNDimSteps(const std::array<int, N_DIM> &dims_size);

        /**
         * @brief Get the Data Size
         *
         * @param dims_size Input the size of dimension
         * @return ErrorType
         */
        ErrorType SetDataSize(const std::array<int, N_DIM> &dims_size);

        std::array<int, N_DIM> dims_size_;
        std::array<int, N_DIM> dims_step_;
        std::array<double, N_DIM> dims_resolution_;
        std::array<std::string, N_DIM> dims_name_;
        std::array<double, N_DIM> origin_;

        int data_size_{0};
        std::vector<T> data_;
    };

    struct LaneRaw {
        int id;
        int dir;

        std::vector<int> child_id;
        std::vector<int> father_id;

        int l_lane_id;
        bool l_change_avbl;
        int r_lane_id;
        bool r_change_avbl;

        std::string behavior;
        double length;

        Eigen::Matrix<double,2,1> start_point;
        Eigen::Matrix<double,2,1> final_point;
        vec_E<Eigen::Matrix<double,2,1>> lane_points;

        /**
         * @brief Print info
         */
        void print() const;
    };

    struct LaneNet {
        std::unordered_map<int, LaneRaw> lane_set;

        /**
         * @brief Clear the container
         */
        inline void clear() { lane_set.clear(); }

        /**
         * @brief Print info
         */
        void print() const;
    };

    struct SemanticLane {
        int id;
        int dir;

        std::vector<int> child_id;
        std::vector<int> father_id;

        int l_lane_id;
        bool l_change_avbl;
        int r_lane_id;
        bool r_change_avbl;

        std::string behavior;
        double length;

        Lane lane;
    };

    struct SemanticLaneSet {
        std::unordered_map<int, SemanticLane> semantic_lanes;

        /**
         * @brief Return the size of container
         *
         * @return int size
         */
        inline int size() const { return semantic_lanes.size(); }

        /**
         * @brief Clear the container
         */
        void clear() { semantic_lanes.clear(); }

        /**
         * @brief Print info
         */
        void print() const;
    };

    struct CircleObstacle {
        int id;
        int type = 0;
        Circle circle;

        /**
         * @brief Print info
         */
        void print() const;
    };

    struct PolygonObstacle {
        int id;
        int type = 0;
        Polygon polygon;

        /**
         * @brief Print info
         */
        void print() const;
    };

    struct ObstacleSet {
        std::unordered_map<int, CircleObstacle> obs_circle;
        std::unordered_map<int, PolygonObstacle> obs_polygon;

        /**
         * @brief Return the size of container
         *
         * @return int size
         */
        inline int size() const { return obs_circle.size() + obs_polygon.size(); }

        /**
         * @brief Print info
         */
        void print() const;
    };

/**
 * @brief Data structure for Nanoflann
 */
    struct PointVecForKdTree {
        std::vector<PointWithValue<int>> pts;

        /**
         * @brief Get the container size
         *
         * @return size_t Output size
         */
        inline size_t kdtree_get_point_count() const { return pts.size(); }

        /**
         * @brief Get value of desired dimension of ith point
         *
         * @param idx Index of point
         * @param dim Dimension
         * @return double Output value
         */
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return dim == 0 ? pts[idx].pt.x : pts[idx].pt.y;
        }

        /**
         * @brief Optional bounding-box computation, not used here
         *
         * @tparam BBOX
         * @return true
         * @return false
         */
        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const {
            return false;
        }
    };

    template <int N_DIM>
    struct SpatioTemporalSemanticCubeNd {
        double t_lb, t_ub;
        std::array<double, N_DIM> p_lb, p_ub;
        std::array<double, N_DIM> v_lb, v_ub;
        std::array<double, N_DIM> a_lb, a_ub;

        SpatioTemporalSemanticCubeNd() { FillDefaultBounds(); }

        void FillDefaultBounds() {
            // ~ for optimization, infinity bound will cause numerical
            // ~ unstable. So we put some loose bounds by default
            t_lb = 0.0;
            t_ub = 1.0;

            const double default_pos_lb = -1;
            const double default_pos_ub = 1;
            const double default_vel_lb = -50.0;
            const double default_vel_ub = 50.0;
            const double default_acc_lb = -20.0;
            const double default_acc_ub = 20.0;

            p_lb.fill(default_pos_lb);
            v_lb.fill(default_vel_lb);
            a_lb.fill(default_acc_lb);

            p_ub.fill(default_pos_ub);
            v_ub.fill(default_vel_ub);
            a_ub.fill(default_acc_ub);
        }
    };

    struct DrivingCube {
        vec_E<Eigen::Matrix<int,3,1>> seeds;
        AxisAlignedCubeNd<int, 3> cube;
    };

    struct DrivingCorridor {
        int id;
        bool is_valid;
        vec_E<DrivingCube> cubes;
    };

    class TrafficSignal {
    public:
        TrafficSignal();
        TrafficSignal(const Eigen::Matrix<double,2,1> &start_point, const Eigen::Matrix<double,2,1> &end_point,
                      const Eigen::Matrix<double,2,1> &valid_time, const Eigen::Matrix<double,2,1> &vel_range);
        void set_start_point(const Eigen::Matrix<double,2,1> &start_point);
        void set_end_point(const Eigen::Matrix<double,2,1> &end_point);
        void set_valid_time_til(const double max_valid_time);
        void set_valid_time_begin(const double min_valid_time);
        void set_valid_time(const Eigen::Matrix<double,2,1> &valid_time);
        void set_vel_range(const Eigen::Matrix<double,2,1> &vel_range);
        void set_lateral_range(const Eigen::Matrix<double,2,1> &lateral_range);
        void set_max_velocity(const double max_velocity);

        void set_start_angle(const double angle) { start_angle_ = angle; }
        void set_end_angle(const double angle) { end_angle_ = angle; }

        Eigen::Matrix<double,2,1> start_point() const;
        Eigen::Matrix<double,2,1> end_point() const;
        Eigen::Matrix<double,2,1> valid_time() const;
        Eigen::Matrix<double,2,1> vel_range() const;
        Eigen::Matrix<double,2,1> lateral_range() const;
        double max_velocity() const;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        double start_angle() const { return start_angle_; }
        double end_angle() const { return end_angle_; }

    protected:
        Eigen::Matrix<double,2,1> start_point_;          // 2d point in x-y plane
        double start_angle_ = 0;  // ! (lu.zhang) Temp, for vis
        Eigen::Matrix<double,2,1> end_point_;            // 2d point in x-y plane
        double end_angle_ = 0;    // ! (lu.zhang) Temp, for vis
        Eigen::Matrix<double,2,1> valid_time_;           // time stamp [stamp_begin, stamp_end]
        Eigen::Matrix<double,2,1> vel_range_;            // velocity [lower_bound, upper_bound]
        Eigen::Matrix<double,2,1> lateral_range_;
    };

    class SpeedLimit : public TrafficSignal {
    public:
        SpeedLimit(const Eigen::Matrix<double,2,1> &start_point, const Eigen::Matrix<double,2,1> &end_point,
                   const Eigen::Matrix<double,2,1> &vel_range);
    };

    class StoppingSign : public TrafficSignal {
    public:
        StoppingSign(const Eigen::Matrix<double,2,1> &start_point, const Eigen::Matrix<double,2,1> &end_point);
    };

    class TrafficLight : public TrafficSignal {
    public:
        enum Type { Green = 0, Red, Yellow, RedYellow };
        void set_type(const Type &type);
        Type type() const;

    private:
        Type type_;
    };

    class SemanticsUtils {
    public:
        /**
         * @brief Return the name of longitudinal behavior
         *
         * @param behavior
         * @return std::string
         */
        static std::string RetLonBehaviorName(const LongitudinalBehavior b) {
            std::string b_str;
            switch (b) {
                case LongitudinalBehavior::kMaintain: {
                    b_str = std::string("M");
                    break;
                }
                case LongitudinalBehavior::kAccelerate: {
                    b_str = std::string("A");
                    break;
                }
                case LongitudinalBehavior::kDecelerate: {
                    b_str = std::string("D");
                    break;
                }
                case LongitudinalBehavior::kStopping: {
                    b_str = std::string("S");
                    break;
                }
                default: {
                    b_str = std::string("Null");
                    break;
                }
            }
            return b_str;
        }

        /**
         * @brief Return the name of lateral behavior
         *
         * @param behavior
         * @return std::string
         */
        static std::string RetLatBehaviorName(const LateralBehavior b) {
            std::string b_str;
            switch (b) {
                case LateralBehavior::kUndefined: {
                    b_str = std::string("U");
                    break;
                }
                case LateralBehavior::kLaneKeeping: {
                    b_str = std::string("K");
                    break;
                }
                case LateralBehavior::kLaneChangeLeft: {
                    b_str = std::string("L");
                    break;
                }
                case LateralBehavior::kLaneChangeRight: {
                    b_str = std::string("R");
                    break;
                }
                default: {
                    b_str = std::string("Null");
                    break;
                }
            }
            return b_str;
        }

        /**
         * @brief Get the Oriented Bounding Box For Vehicle Using State object
         *
         * @param param Vehicle parameter
         * @param s Vehicle state
         * @param obb Output Vehicle OBB
         * @return ErrorType
         */
        static ErrorType GetOrientedBoundingBoxForVehicleUsingState(
                const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb);

        /**
         * @brief Get the Vehicle Vertices
         *
         * @param param Vehicle parameter
         * @param state Vehicle state
         * @param vertices Output vertice vector
         * @return ErrorType
         */
        static ErrorType GetVehicleVertices(const VehicleParam &param,
                                            const State &state,
                                            vec_E<Eigen::Matrix<double,2,1>> *vertices);

        static ErrorType InflateVehicleBySize(const Vehicle &vehicle_in,
                                              const double delta_w,
                                              const double delta_l,
                                              Vehicle *vehicle_out);

    };  // SemanticsUtils
}

#endif //SRC_SEMANTICS_H
