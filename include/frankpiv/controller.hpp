#ifndef FRANKPIV_CONTROLLER
#define FRANKPIV_CONTROLLER

#include "frankpiv/general_backend.hpp"

namespace frankpiv {
    class Controller {
    private:
        frankpiv::backend::GeneralBackend *backend;
    public:

        explicit Controller(const std::string &config_file);

        ~Controller();

        void start(Eigen::Affine3d *reference_frame = nullptr) const;
        void start(std::optional<std::array<double, 6>> reference_frame) const;

        void stop() const;

        void moveToPoint(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr) const;
        void moveToPoint(std::array<double, 3> point, double roll, std::optional<std::array<double, 6>> frame) const;

        void movePYRZ(const Eigen::Vector4d &pyrz, bool degrees = false) const;
        void movePYRZ(std::array<double, 4> pyrz, bool degrees = false) const;

        void movePYRZRelative(const Eigen::Vector4d &pyrz, bool degrees = false) const;
        void movePYRZRelative(std::array<double, 4> pyrz, bool degrees = false) const;

        [[nodiscard]] double getInitialEefPpointDistance() const;

        [[nodiscard]] double getToolLength() const;

        [[nodiscard]] double getMaxAngle() const;

        [[nodiscard]] const Eigen::Vector2d &getRollBoundaries() const;
        [[nodiscard]] std::array<double, 2> getRollBoundariesAsArray() const;

        [[nodiscard]] const Eigen::Vector2d &getZTranslationBoundaries() const;
        [[nodiscard]] std::array<double, 2> getZTranslationBoundariesAsArray() const;

        [[nodiscard]] bool isClipToBoundaries() const;

        [[nodiscard]] bool isMoveDirectly() const;

        [[nodiscard]] const Eigen::Affine3d &getReferenceFrame() const;
        [[nodiscard]] std::array<double, 6> getReferenceFrameAsArray() const;

        [[nodiscard]] Eigen::Vector4d getCurrentPyrz();
        [[nodiscard]] std::array<double, 4> getCurrentPyrzAsArray();

        [[nodiscard]] double getPivotError();

#ifdef VISUALIZATION

        [[nodiscard]] bool isVisualize() const;

        void setVisualize(bool visualize_);

#endif
        void setInitialEefPpointDistance(double initialEefPpointDistance);

        void setToolLength(double toolLength);

        void setMaxAngle(double maxAngle);

        void setRollBoundaries(const Eigen::Vector2d &rollBoundaries);
        void setRollBoundaries(const std::array<double, 2> &rollBoundaries);

        void setZTranslationBoundaries(const Eigen::Vector2d &zTranslationBoundaries);
        void setZTranslationBoundaries(const std::array<double, 2> &zTranslationBoundaries);

        void setClipToBoundaries(bool clipToBoundaries);

        void setMoveDirectly(bool moveDirectly);

        void setReferenceFrame(const Eigen::Affine3d &referenceFrame);
        void setReferenceFrame(const std::array<double, 6> &referenceFrame);
    };
}

#endif //FRANKPIV_CONTROLLER_H