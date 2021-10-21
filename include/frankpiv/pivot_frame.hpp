#ifndef FRANKPIV_PIVOT_FRAME
#define FRANKPIV_PIVOT_FRAME

#include <Eigen/Geometry>

namespace frankpiv {
    //TODO WRITE UNIT TESTS FOR THIS CLASS !11!1
    class PivotFrame {
    public:
        PivotFrame(Eigen::Affine3d reference_frame);

        Eigen::Vector4d getPYRZ(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Vector4d getPYRZ(const Eigen::Affine3d &pose, const double *error_tolerance = nullptr, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Affine3d getPose(const Eigen::Vector4d &pyrz, const Eigen::Affine3d *frame = nullptr) const;

        double getError(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        static double getAngle(const Eigen::Vector4d &pyrz);

        static bool clipAngle(Eigen::Vector4d &pyrz, double max_angle);

        [[nodiscard]] Eigen::Affine3d reference_frame() const { return reference_frame_; }

        void reference_frame(Eigen::Affine3d reference_frame) { reference_frame_ = std::move(reference_frame); }

    private:
        Eigen::Affine3d reference_frame_;

        Eigen::Affine3d otherToLocalFrame(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Vector3d otherToLocalFrame(const Eigen::Vector3d &point, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Affine3d localToOtherFrame(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        static Eigen::Vector4d internalPoseToPYRZ(Eigen::Affine3d pose);

        static Eigen::Vector4d getPYRZInternal(const Eigen::Vector3d &point, double roll);

        static Eigen::Affine3d getPoseInternal(const Eigen::Vector4d &pyrz);

        static double getErrorInternal(const Eigen::Affine3d &pose);

        static double getAngleInternal(const Eigen::Vector4d &pyrz, Eigen::Vector3d *point = nullptr, Eigen::Vector3d *z_axis = nullptr, double *z_translation = nullptr);
    };
}
#endif //FRANKPIV_PIVOT_FRAME
