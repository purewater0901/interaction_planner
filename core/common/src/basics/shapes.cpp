#include "basics/shapes.h"

namespace common
{

    Point::Point() {}

    Point::Point(double _x, double _y) : x(_x), y(_y) {}

    Point::Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    void Point::print() const { printf("(%f, %f)", x, y); }

    Point2i::Point2i() {}

    Point2i::Point2i(int _x, int _y) : x(_x), y(_y) {}

    void Point2i::print() const { printf("(%d, %d)", x, y); }

    OrientedBoundingBox2D::OrientedBoundingBox2D() {}

    OrientedBoundingBox2D::OrientedBoundingBox2D(const double x_,
                                                 const double y_,
                                                 const double angle_,
                                                 const double width_,
                                                 const double length_)
            : x(x_), y(y_), angle(angle_), width(width_), length(length_) {}

    void Circle::print() const
    {
        printf("Circle:\n");
        printf(" -- center:");
        center.print();
        printf("\n -- radius: %lf\n", radius);
    }

    void PolyLine::print() const
    {
        printf("PolyLine:\n");
        printf(" -- dir: %d", dir);
        printf(" -- num of pts: %d", (int)points.size());
    }

    void Polygon::print() const
    {
        printf("Polygon:\n");
        printf(" -- num of pts: %d", (int)points.size());
    }

    bool ShapeUtils::CheckIfOrientedBoundingBoxIntersect(const OrientedBoundingBox2D& obb_a,
                                                         const OrientedBoundingBox2D& obb_b)
    {
        vec_E<Eigen::Matrix<double, 2, 1>> vertices_a, vertices_b;
        GetVerticesOfOrientedBoundingBox(obb_a, &vertices_a);
        GetVerticesOfOrientedBoundingBox(obb_b, &vertices_b);
        vec_E<Eigen::Matrix<double, 2, 1>> axes;
        GetPerpendicularAxesOfOrientedBoundingBox(vertices_a, &axes);
        GetPerpendicularAxesOfOrientedBoundingBox(vertices_b, &axes);
        Eigen::Matrix<double, 2, 1> proj_a, proj_b;
        double overlap_len;
        for (auto& axis : axes)
        {
            GetProjectionOnAxis(vertices_a, axis, &proj_a);
            GetProjectionOnAxis(vertices_b, axis, &proj_b);
            GetOverlapLength(proj_a, proj_b, &overlap_len);
            if (fabs(overlap_len) < constants::kEPS)   // shapes are not overlapping
                return false;
        }
        return true;
    }

    ErrorType ShapeUtils::GetVerticesOfOrientedBoundingBox(const OrientedBoundingBox2D& obb,
                                                           vec_E<Eigen::Matrix<double, 2, 1>>* vertices)
    {
        //
        //    corner2   corner1
        // y<--   ________   x
        //        |      |   ^
        //        |      |   |
        //        |      |
        //        |______|
        //    corner3   corner4

        vertices->clear();
        vertices->reserve(4);
        double cos_theta = cos(obb.angle);
        double sin_theta = sin(obb.angle);
        Eigen::Matrix<double, 2, 1> corner1(
                obb.x + 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta,
                obb.y + 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
        Eigen::Matrix<double, 2, 1> corner2(
                obb.x + 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta,
                obb.y + 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
        Eigen::Matrix<double, 2, 1> corner3(
                obb.x - 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta,
                obb.y - 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
        Eigen::Matrix<double, 2, 1> corner4(
                obb.x - 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta,
                obb.y - 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
        vertices->push_back(corner1);
        vertices->push_back(corner2);
        vertices->push_back(corner3);
        vertices->push_back(corner4);

        return kSuccess;
    }

    ErrorType ShapeUtils::GetPerpendicularAxesOfOrientedBoundingBox(const vec_E<Eigen::Matrix<double, 2, 1>>& vertices,
                                                                    vec_E<Eigen::Matrix<double, 2, 1>>* axes)
    {
        // for obb 2d, two axes are enough
        Eigen::Matrix<double, 2, 1> axis0, axis1;
        GetPerpendicularAxisOfOrientedBoundingBox(vertices, 0, &axis0);
        GetPerpendicularAxisOfOrientedBoundingBox(vertices, 1, &axis1);
        axes->push_back(axis0);
        axes->push_back(axis1);

        return kSuccess;
    }

    ErrorType ShapeUtils::GetProjectionOnAxis(const vec_E<Eigen::Matrix<double, 2, 1>>& vertices,
                                              const Eigen::Matrix<double, 2, 1>& axis,
                                              Eigen::Matrix<double, 2, 1>* proj)
    {
        double min = std::numeric_limits<double>::infinity();
        double max = -std::numeric_limits<double>::infinity();
        double projection;
        for (auto& vertex : vertices)
        {
            projection = vertex.dot(axis);
            if (projection < min)
                min = projection;
            if (projection > max)
                max = projection;
        }
        *proj = Eigen::Matrix<double, 2, 1>(min, max);

        return kSuccess;
    }

    ErrorType ShapeUtils::GetPerpendicularAxisOfOrientedBoundingBox(const vec_E<Eigen::Matrix<double, 2, 1>>& vertices,
                                                                    const int& index, Eigen::Matrix<double, 2, 1>* axis)
    {
        assert(index >= 0 && index < 4);
        Eigen::Matrix<double, 2, 1> vec = vertices[index + 1] - vertices[index];
        double length = vec.norm();
        Eigen::Matrix<double, 2, 1> normalized_vec = Eigen::Matrix<double, 2, 1>::Zero();
        if (length > constants::kEPS) normalized_vec = vec / length;
        // right hand normal vector
        (*axis)[0] = -normalized_vec[1];
        (*axis)[1] = normalized_vec[0];
        return kSuccess;
    }

    ErrorType ShapeUtils::GetOverlapLength(const Eigen::Matrix<double, 2, 1>& a,
                                           const Eigen::Matrix<double, 2, 1>& b,
                                           double* len)
    {
        if (a.x() > b.y() || a.y() < b.x())
        {
            *len = 0.0;
            return kSuccess;
        }

        *len = std::min(a.y(), b.y()) - std::max(a.x(), b.x());
        return kSuccess;
}

    ErrorType ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(const std::vector<Point2i>& pts_in,
                                                               std::vector<cv::Point2i>* pts_out)
    {
        int num = pts_in.size();
        pts_out->resize(num);
        for (int i = 0; i < num; ++i)
            GetCvPoint2iUsingCommonPoint2i(pts_in[i], pts_out->data() + i);
        return kSuccess;
    }

    ErrorType ShapeUtils::GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
                                                         cv::Point2i* pt_out)
    {
        pt_out->x = pt_in.x;
        pt_out->y = pt_in.y;
        return kSuccess;
    }

}  // namespace common