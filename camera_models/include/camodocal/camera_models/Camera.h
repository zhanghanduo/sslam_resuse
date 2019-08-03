/*******************************************************
 * Copyright (c) 2013 Lionel Heng
 *
 * \file Camera.h
 * \author Andrew Hundt (ahundt.github.io), Lionel Heng (www.lionel.work)
 * \date July 2015
 * \brief Description of different camera models.
 *
 * This software is licensed under CC-BY-SA. For more information, visit
 * http://http://creativecommons.org/licenses/by-sa/2.0/
 *
 *******************************************************/

#ifndef CAMERA_H
#define CAMERA_H

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

namespace camodocal
{

    class Camera
    {
        public:
        #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

        ///
        /// \brief Common camera models
        ///
        enum ModelType
        {
            KANNALA_BRANDT, // Equidistant model
            MEI,            // Fish-eye large FOV camera (cata camera)
            PINHOLE,
            PINHOLE_FULL,
            SCARAMUZZA      // Omnidirectional Camera
        };

        class Parameters
        {
            public:
            #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */
            Parameters( ModelType modelType );

            Parameters( ModelType modelType, const std::string& cameraName, int w, int h );

            ModelType& modelType( void );
            std::string& cameraName( void );
            int& imageWidth( void );
            int& imageHeight( void );

            ModelType modelType( void ) const;
            const std::string& cameraName( void ) const;
            int imageWidth( void ) const;
            int imageHeight( void ) const;

            int nIntrinsics( void ) const;

            virtual bool readFromYamlFile( const std::string& filename )      = 0;
            virtual void writeToYamlFile( const std::string& filename ) const = 0;

            protected:
            ModelType m_modelType;
            int m_nIntrinsics;
            std::string m_cameraName;
            int m_imageWidth;
            int m_imageHeight;
        };

        virtual ModelType modelType( void ) const           = 0;
        virtual const std::string& cameraName( void ) const = 0;
        virtual int imageWidth( void ) const                = 0;
        virtual int imageHeight( void ) const               = 0;

        virtual cv::Mat& mask( void );
        virtual const cv::Mat& mask( void ) const;

        virtual void estimateIntrinsics( const cv::Size& boardSize,
                                         const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                         const std::vector< std::vector< cv::Point2f > >& imagePoints )
        = 0;
        virtual void estimateExtrinsics( const std::vector< cv::Point3f >& objectPoints,
                                         const std::vector< cv::Point2f >& imagePoints,
                                         cv::Mat& rvec,
                                         cv::Mat& tvec ) const;

        // Lift points from the image plane to the sphere
        virtual void liftSphere( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const = 0;
        //%output P

        // Lift points from the image plane to the projective space
        virtual void liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const = 0;
        //%output P

        /**
        * \brief Virual function: Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
        *
        * \param P 3D point coordinates
        * \param p return value, contains the image point coordinates
        */
        virtual void spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p ) const = 0;
        //%output p

        // Projects 3D points to the image plane (Pi function)
        // and calculates jacobian
        // virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
        //                          Eigen::Matrix<double,2,3>& J) const = 0;
        //%output p
        //%output J

        virtual void undistToPlane( const Eigen::Vector2d& p_u, Eigen::Vector2d& p ) const = 0;
        //%output p

        // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0)
        // const = 0;
        virtual cv::Mat initUndistortRectifyMap( cv::Mat& map1,
                                                 cv::Mat& map2,
                                                 float fx           = -1.0f,
                                                 float fy           = -1.0f,
                                                 cv::Size imageSize = cv::Size( 0, 0 ),
                                                 float cx           = -1.0f,
                                                 float cy           = -1.0f,
                                                 cv::Mat rmat = cv::Mat::eye( 3, 3, CV_32F ) ) const = 0;

        virtual int parameterCount( void ) const = 0;

        virtual void readParameters( const std::vector< double >& parameters )  = 0;
        virtual void writeParameters( std::vector< double >& parameters ) const = 0;

        virtual void writeParametersToYamlFile( const std::string& filename ) const = 0;

        virtual std::string parametersToString( void ) const = 0;

        /**
         * \brief Calculates the reprojection distance between points.
         *
         * \param P1 first 3D point coordinates.
         * \param P2 second 3D point coordinates.
         * \return euclidean distance in the plane.
         */
        double reprojectionDist( const Eigen::Vector3d& P1, const Eigen::Vector3d& P2 ) const;

        /**
         * \brief Calculates the reprojection error between 3D points and 2D image points.
         *
         * \param P1 vectors of 3D points in world coordinates.
         * \param P2 vectors of 2D points in image coordinate.
         * \param rvec Rotation matrix from world to local camera coordinate.
         * \param tvec Translation matrix from world to local camera coordinate.
         * \param [out] perViewErrors (Optional) visualization of error in cv::Mat format.
         * \return euclidean distance in the plane.
         */
        double reprojectionError( const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                  const std::vector< std::vector< cv::Point2f > >& imagePoints,
                                  const std::vector< cv::Mat >& rvecs,
                                  const std::vector< cv::Mat >& tvecs,
                                  cv::OutputArray perViewErrors = cv::noArray( ) ) const;

        double reprojectionError( const Eigen::Vector3d& P,
                                  const Eigen::Quaterniond& camera_q,
                                  const Eigen::Vector3d& camera_t,
                                  const Eigen::Vector2d& observed_p ) const;

        /**
         * \brief // project 3D object points to the image plane.
         *
         * \param P1 vector of 3D points in world coordinate.
         * \param P2 vector of 2D points in image coordinate.
         * \param rvec Rotation matrix from world to local camera coordinate.
         * \param tvec Translation matrix from world to local camera coordinate .        *
         * \param [out] imagePoints return corresponding 2D image points.
         */
        void projectPoints( const std::vector< cv::Point3f >& objectPoints,
                            const cv::Mat& rvec,
                            const cv::Mat& tvec,
                            std::vector< cv::Point2f >& imagePoints ) const;

        protected:
        cv::Mat m_mask;
    };

    typedef boost::shared_ptr< Camera > CameraPtr;
    typedef boost::shared_ptr< const Camera > CameraConstPtr;
}

#endif
