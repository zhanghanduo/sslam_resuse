
#ifndef PROJECT_MAT_CEREALISATION_H
#define PROJECT_MAT_CEREALISATION_H

#include "cereal/cereal.hpp"
#include "opencv2/core/core.hpp"
//#include <cereal/archives/json.hpp>
/**
 * @brief Serialisation of OpenCV \c cv::Mat matrices for the serialisation
 * library cereal (http://uscilab.github.io/cereal/index.html).
 *
 * Contains serialisation for \c cv::Mat matrices to binary archives, and
 * serialisation of cv::Vec2f.
 *
 * Contains also an experimental serialisation to save/load cv::Mat's from JSON.
 */
namespace cereal {

/**
 * @brief Serialise a cv::Mat using cereal.
 *
 * Supports all types of matrices as well as non-contiguous ones.
 *
 * @param[in] ar The archive to serialise to.
 * @param[in] mat The matrix to serialise.
 */
    template<class Archive, cereal::traits::DisableIf<cereal::traits::is_text_archive<Archive>::value>
    = cereal::traits::sfinae>
    void save(Archive& ar, const cv::Mat& mat)
    {
        int rows, cols, type;
        bool continuous;

        rows = mat.rows;
        cols = mat.cols;
        type = mat.type();
        continuous = mat.isContinuous();

        ar & rows & cols & type & continuous;

        if (continuous) {
            const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            auto mat_data = cereal::binary_data(mat.ptr(), data_size);
            ar & mat_data;
        }
        else {
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                ar & row_data;
            }
        }
    };

/**
 * @brief De-serialise a cv::Mat using cereal.
 *
 * Supports all types of matrices as well as non-contiguous ones.
 *
 * @param[in] ar The archive to deserialise from.
 * @param[in] mat The matrix to deserialise into.
 */
    template<class Archive, cereal::traits::DisableIf<cereal::traits::is_text_archive<Archive>::value>
    = cereal::traits::sfinae>
    void load(Archive& ar, cv::Mat& mat)
    {
        int rows, cols, type;
        bool continuous;

        ar & rows & cols & type & continuous;

        if (continuous) {
            mat.create(rows, cols, type);
            const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            auto mat_data = cereal::binary_data(mat.ptr(), data_size);
            ar & mat_data;
        }
        else {
            mat.create(rows, cols, type);
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                ar & row_data;
            }
        }
    };

    /**
 * @brief Serialise a cv::Mat using cereal, for text archives (JSON specifically).
 *
 * Experimental: This specialisation is enabled for text archives (e.g. XML, JSON),
 * and was created to convert the JSON output from the BFM Matlab converter script
 * to a cereal binary model.
 *
 * Notes:
 * - Only for 2-dim matrices, and float values, i.e. CV_32FC1 cv::Mat's.
 *   Actually, _only_ 32FC1, because of the load() method. In fact, maybe we should
 *   store the type, since we're storing cv::Mat's.
 * - Writes the data row-wise to a json array? or a json object?
 *
 * @param[in] ar The archive to serialise to.
 * @param[in] mat The matrix to serialise.
 */
    template <class Archive,
            cereal::traits::EnableIf<cereal::traits::is_text_archive<Archive>::value>
            = cereal::traits::sfinae>
    void save(Archive& ar, const cv::Mat& mat)
    {
        int rows, cols, type;
        bool continuous;

        rows = mat.rows;
        cols = mat.cols;
        type = mat.type();
        continuous = mat.isContinuous();

        //ar & rows & cols & type & continuous;
        assert(mat.dims == 2); // correct?

        if (continuous) { // We go row by row anyway so no need for this distinction?
            //const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            //std::vector<float> test(mat.begin<float>(), mat.end<float>());
            //ar & test;

            std::vector<std::vector<float>> mat_data;
            for (int i = 0; i < rows; i++) {
                cv::Mat this_row = mat.row(i); // need a temporary, otherwise goes up to 8GB RAM usage
                mat_data.push_back(std::vector<float>(this_row.begin<float>(), this_row.end<float>()));
            }
            ar & cereal::make_nvp("data", mat_data); // Can we somehow not give this a name and make it like the "root" node, part of the parent object? Maybe look at the std::string serialisation?
        }
        else {
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                //auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                //ar & row_data;
            }
        }
    };


/**
 * @brief De-serialise a cv::Mat using cereal, for text archives (JSON specifically).
 *
 * Experimental: This specialisation is enabled for text archives (e.g. XML, JSON),
 * and was created to convert the JSON output from the BFM Matlab converter script
 * to a cereal binary model.
 * See the notes of the save() method!
 *
 * @param[in] ar The archive to deserialise from.
 * @param[in] mat The matrix to deserialise into.
 */
    template <class Archive,
            cereal::traits::EnableIf<cereal::traits::is_text_archive<Archive>::value>
            = cereal::traits::sfinae>
    void load(Archive& ar, cv::Mat& mat)
    {
        //int rows, cols, type;
        //bool continuous;

        //ar & rows & cols & type & continuous;

        std::vector<std::vector<float>> mat_data;
        ar & mat_data;
        assert(mat_data.size() > 0); // hmm can't store empty cv::Mat's... not so nice. Will create problems with SFM shape-only models?
        int rows = static_cast<int>(mat_data.size());
        int cols = static_cast<int>(mat_data[0].size());
        mat.create(rows, cols, CV_32FC1);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                mat.at<float>(r, c) = mat_data[r][c];
            }
        }

/*	mat.create(rows, cols, type);
	const int data_size = rows * cols * static_cast<int>(mat.elemSize());
	auto mat_data = cereal::binary_data(mat.ptr(), data_size);
	ar & mat_data;

	mat.create(rows, cols, type);
	const int row_size = cols * static_cast<int>(mat.elemSize());
	for (int i = 0; i < rows; i++) {
		auto row_data = cereal::binary_data(mat.ptr(i), row_size);
		ar & row_data;
	}*/
    };


    /**
     * @brief Serialisation of a cv::Vec2f using cereal.
     *
     * @param[in] ar The archive to (de)serialise.
     * @param[in] vec The vector to (de)serialise.
     */
    template <class Archive>
    void serialize(Archive& ar, cv::Vec2f& vec)
    {
        ar(vec[0], vec[1]);
    };

     /**
     * @brief Serialisation of a cv::Vec3f using cereal.
     *
     * @param[in] ar The archive to (de)serialise.
     * @param[in] vec The vector to (de)serialise.
     */
    template <class Archive>
    void serialize(Archive& ar, cv::Vec3f& vec)
    {
        ar(vec[0], vec[1], vec[2]);
    };

    /**
    * @brief Serialisation of a cv::KeyPoint using cereal.
    *
    * @param[in] ar The archive to (de)serialise.
    * @param[in] vec The KeyPoint to (de)serialise.
    */
    /* serialization for CV KeyPoint */
    template<class Archive>
    inline void serialize(Archive& ar, cv::KeyPoint& kf)
    {
        ar(kf.angle, kf.class_id, kf.octave, kf.response, kf.pt.x, kf.pt.y);
    }

    // cv::Point
    template<class Archive>
    inline void serialize(Archive& ar, cv::Point& pt)
    {
        ar(pt.x, pt.y);
    }

    // cv::Point3f
    template<class Archive>
    inline void serialize(Archive& ar, cv::Point3f& pt)
    {
        ar(pt.x, pt.y, pt.z);
    }


    // cv::DMatch
    template<class Archive>
    inline void serialize(Archive& ar, cv::DMatch& match)
    {
        ar(match.queryIdx, match.trainIdx, match.imgIdx, match.distance);
    }

} /* namespace cereal */

#endif //PROJECT_MAT_CEREALISATION_H
