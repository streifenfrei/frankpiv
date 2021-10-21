#ifndef FRANKPIV_CONTROLLER
#define FRANKPIV_CONTROLLER

#include "frankpiv/utilities.hpp"
#include "frankpiv/general_backend.hpp"

using namespace frankpiv::util;

namespace frankpiv {
    class Controller {
    private:
        std::unique_ptr <frankpiv::backend::GeneralBackend> backend;

    public:

        explicit Controller(const std::string &config_file);

        ~Controller();

        void start(Eigen::Affine3d *reference_frame = nullptr) const;

        void start(std::optional <std::array<double, 6>> reference_frame) const;

        void stop() const;

        void moveToPoint(Eigen::Vector3d point, double roll, const Eigen::Affine3d *frame = nullptr) const;

        void moveToPoint(std::array<double, 3> point, double roll, std::optional <std::array<double, 6>> frame) const;

        void movePYRZ(Eigen::Vector4d pyrz, bool degrees = false) const;

        void movePYRZ(std::array<double, 4> pyrz, bool degrees = false) const;

        void movePYRZRelative(Eigen::Vector4d pyrz, bool degrees = false) const;

        void movePYRZRelative(std::array<double, 4> pyrz, bool degrees = false) const;

        [[nodiscard]] Eigen::Vector4d getCurrentPYRZ() const;

        [[nodiscard]] std::array<double, 4> getCurrentPYRZAsArray() const;

        [[nodiscard]] double initial_eef_ppoint_distance() const { return (*this->backend).initial_eef_ppoint_distance(); }

        void initial_eef_ppoint_distance(double initial_eef_ppoint_distance) { (*this->backend).initial_eef_ppoint_distance(initial_eef_ppoint_distance); }

        [[nodiscard]] double tool_length() const { return (*this->backend).tool_length(); }

        void tool_length(double tool_length) { (*this->backend).tool_length(tool_length); }

        [[nodiscard]] double error_tolerance() const { return (*this->backend).error_tolerance(); }

        void error_tolerance(double error_tolerance) { (*this->backend).error_tolerance(error_tolerance); }

        [[nodiscard]] double max_angle() const { return (*this->backend).max_angle(); }

        void max_angle(double max_angle) { (*this->backend).max_angle(max_angle); }

        [[nodiscard]] Eigen::Vector2d roll_boundaries() const { return (*this->backend).roll_boundaries(); }

        [[nodiscard]] std::array<double, 2> roll_boundaries_as_array() const {
            Eigen::Vector2d boundaries = this->roll_boundaries();
            return {boundaries[0], boundaries[1]};
        }

        void roll_boundaries(Eigen::Vector2d roll_boundaries) { (*this->backend).roll_boundaries(roll_boundaries); };

        void roll_boundaries(std::array<double, 2> roll_boundaries) {
            Eigen::Vector2d boundaries = Eigen::Vector2d(roll_boundaries[0], roll_boundaries[1]);
            this->roll_boundaries(boundaries);
        };

        [[nodiscard]] Eigen::Vector2d z_translation_boundaries() const { return (*this->backend).z_translation_boundaries(); }

        [[nodiscard]] std::array<double, 2> z_translation_boundaries_as_array() const {
            Eigen::Vector2d boundaries = this->z_translation_boundaries();
            return {boundaries[0], boundaries[1]};
        }

        void z_translation_boundaries(Eigen::Vector2d z_translation_boundaries) { (*this->backend).z_translation_boundaries(z_translation_boundaries); };

        void z_translation_boundaries(std::array<double, 2> z_translation_boundaries) {
            Eigen::Vector2d boundaries = Eigen::Vector2d(z_translation_boundaries[0], z_translation_boundaries[1]);
            this->z_translation_boundaries(boundaries);
        };

        [[nodiscard]] bool clip_to_boundaries() const { return (*this->backend).clip_to_boundaries(); }

        void clip_to_boundaries(bool clip_to_boundaries) { (*this->backend).clip_to_boundaries(clip_to_boundaries); }

        [[nodiscard]] bool move_directly() const { return (*this->backend).move_directly(); }

        void move_directly(bool move_directly) { (*this->backend).move_directly(move_directly); }

        [[nodiscard]] Eigen::Affine3d reference_frame() const { return (*this->backend).reference_frame(); }

        [[nodiscard]] std::array<double, 6> reference_frame_as_array() const {
            Eigen::Affine3d reference_frame = this->reference_frame();
            return toArray(reference_frame);
        }

        void reference_frame(Eigen::Affine3d reference_frame) { (*this->backend).reference_frame(reference_frame); }

        void reference_frame(std::array<double, 6> reference_frame) {
            Eigen::Affine3d affine = toAffine(reference_frame);
            this->reference_frame(affine);
        }

#ifdef VISUALIZATION
        [[nodiscard]] bool visualize() const { return (*this->backend).visualize(); }
        void visualize(bool visualize) { (*this->backend).visualize(visualize); }
#endif
    };
}

#endif //FRANKPIV_CONTROLLER_H