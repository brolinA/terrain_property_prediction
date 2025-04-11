#ifndef DATA_NORMALIZER_HPP
#define DATA_NORMALIZER_HPP
/**
 * @file data_normalizer.hpp
 * @brief Defines the DataNormalizer struct for normalizing data using a circular buffer.
 */

#include <boost/circular_buffer.hpp>
#include <vector>
#include <algorithm>

/**
 * @struct DataNormalizer
 * @brief A utility for normalizing data using a sliding window approach with a circular buffer.
 * 
 * This struct maintains a buffer of recent data points and provides functionality to normalize
 * incoming data based on the minimum and maximum values in the buffer. It also allows retrieval
 * of normalized data and normalization parameters.
 */
struct DataNormalizer
{
    boost::circular_buffer<double> dataBuffer; ///< Circular buffer to store recent data points.
    double minValue; ///< Minimum value in the current buffer.
    double maxValue; ///< Maximum value in the current buffer.
    std::vector<double> normalizedData; ///< Vector to store normalized data points.

    /**
     * @brief Constructs a DataNormalizer with the specified buffer size and initial min/max values.
     * @param bufferSize The size of the circular buffer.
     * @param minVal The initial minimum value.
     * @param maxVal The initial maximum value.
     */
    DataNormalizer(size_t bufferSize=100, double minVal=std::numeric_limits<double>::max(), 
                    double maxVal=-1)
        : dataBuffer(bufferSize), minValue(minVal), maxValue(maxVal) {}

    /**
     * @brief Adds a new data point to the buffer.
     * @param value The data point to add.
     */
    void addData(double value)
    {
        dataBuffer.push_back(value);
    }

    /**
     * @brief Normalizes a given data point based on the current buffer's min and max values.
     * 
     * This function updates the buffer, recalculates the min and max values, and normalizes
     * the input data point.
     * 
     * @param inputData The data point to normalize.
     * @return The normalized value of the input data.
     */
    double normalizeData(double inputData)
    {
        dataBuffer.push_back(inputData);

        // Update min and max values based on the current buffer
        updateMinMax();

        // Normalize the input data
        double normalizedValue = ((inputData - minValue) / (maxValue - minValue));
        normalizedData.push_back(normalizedValue);

        return normalizedValue;
    }

    /**
     * @brief Resets the buffer and clears all stored data.
     */
    void resetBuffer()
    {
        dataBuffer.clear();
        normalizedData.clear();
    }
    
    /**
     * @brief Checks if the buffer is full and ready for normalization.
     * @return True if the buffer is full, false otherwise.
     */
    bool isDataReady()
    {
        return dataBuffer.size() == dataBuffer.capacity();
    }

    /**
     * @brief Retrieves the current normalization parameters (min and max values).
     * @param minVal Reference to store the current minimum value.
     * @param maxVal Reference to store the current maximum value.
     */
    void getNormalizationParams(double& minVal, double& maxVal)
    {
        
            minVal = minValue;
            maxVal = maxValue;
        
    }

    /**
     * @brief Retrieves the normalized data stored in the buffer.
     * @param normalizedDataOut Reference to a vector to store the normalized data.
     */
    void getNormalizedData(std::vector<double>& normalizedDataOut) const
    {
        normalizedDataOut = normalizedData;
    }

    /**
     * @brief Updates the minimum and maximum values in the buffer.
     * 
     * This function recalculates the minimum and maximum values based on the current
     * contents of the circular buffer. If the new minimum or maximum values are
     * smaller or larger than the current ones, they are updated accordingly.
     */
    void updateMinMax()
    {
        double newMinValue = *std::min_element(dataBuffer.begin(), dataBuffer.end());
        double newMaxValue = *std::max_element(dataBuffer.begin(), dataBuffer.end());

        if (newMinValue < minValue)
            minValue = newMinValue;
        if (newMaxValue > maxValue)
            maxValue = newMaxValue;
    }
};
#endif // DATA_NORMALIZER_HPP