#ifndef FRANKPIV_PIVOT_FRAME
#define FRANKPIV_PIVOT_FRAME

#include <Eigen/Geometry>

namespace frankpiv {
    //TODO WRITE UNIT TESTS FOR THIS CLASS !11!1
    class PivotFrame {
    public:
        PivotFrame(Eigen::Affine3d reference_frame, double max_angle, Eigen::Vector2d roll_boundaries, Eigen::Vector2d z_translation_boundaries, double error_tolerance);

        Eigen::Vector4d getPYRZ(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Vector4d getPYRZ(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Affine3d getPose(const Eigen::Vector4d &pyrz, const Eigen::Affine3d *frame = nullptr) const;

        double getError(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        bool isValid(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        bool clipAngle(Eigen::Vector4d &pyrz);

        bool clipRoll(Eigen::Vector4d &pyrz);

        bool clipZTranslation(Eigen::Vector4d &pyrz);

        bool clip(Eigen::Vector4d &pyrz);

        static double getAngle(const Eigen::Vector4d &pyrz);

        [[nodiscard]] Eigen::Affine3d reference_frame() const { return reference_frame_; }

        void reference_frame(Eigen::Affine3d reference_frame) { reference_frame_ = std::move(reference_frame); }

        [[nodiscard]] double max_angle() const { return max_angle_; }

        void max_angle(double max_angle) { max_angle_ = max_angle; }

        [[nodiscard]] Eigen::Vector2d roll_boundaries() const { return roll_boundaries_; }

        void roll_boundaries(Eigen::Vector2d roll_boundaries) { roll_boundaries_ = std::move(roll_boundaries); }

        [[nodiscard]] Eigen::Vector2d z_translation_boundaries() const { return z_translation_boundaries_; }

        void z_translation_boundaries(Eigen::Vector2d z_translation_boundaries) { z_translation_boundaries_ = std::move(z_translation_boundaries); }

        [[nodiscard]] double error_tolerance() const { return error_tolerance_; }

        void error_tolerance(double error_tolerance) { error_tolerance_ = error_tolerance; }

    private:
        Eigen::Affine3d reference_frame_;
        double max_angle_;
        Eigen::Vector2d roll_boundaries_;
        Eigen::Vector2d z_translation_boundaries_;
        double error_tolerance_;

        Eigen::Affine3d otherToLocalFrame(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Vector3d otherToLocalFrame(const Eigen::Vector3d &point, const Eigen::Affine3d *frame = nullptr) const;

        Eigen::Affine3d localToOtherFrame(const Eigen::Affine3d &pose, const Eigen::Affine3d *frame = nullptr) const;

        static Eigen::Vector4d internalPoseToPYRZ(Eigen::Affine3d &pose);

        static Eigen::Vector4d getPYRZInternal(const Eigen::Vector3d &point, double roll);

        static Eigen::Affine3d getPoseInternal(const Eigen::Vector4d &pyrz);

        static double getErrorInternal(const Eigen::Affine3d &pose);

        static bool isValidInternal(const Eigen::Affine3d &pose, double error_tolerance);

        static double getAngleInternal(const Eigen::Vector4d &pyrz, Eigen::Vector3d *point = nullptr, Eigen::Vector3d *z_axis = nullptr, double *z_translation = nullptr);
    };
}
#endif //FRANKPIV_PIVOT_FRAME
