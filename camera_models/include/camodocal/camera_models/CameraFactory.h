#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "Camera.h"

namespace camodocal
{

class CameraFactory
{
public:
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif
