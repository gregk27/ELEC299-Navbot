#ifndef SMOOTHEDVAL_H
#define SMOOTHEDVAL_H

/**
 * Class used to smooth input data
*/
template <class T>
class SmoothedValue {
  private:
    /** Array holing samples */
    T* samples;
    /** Number of samples in the array */
    int numSamples;
    /** Index of last sample */
    int lastIdx;
    /** Callback function to use in calculating average */
    T (*calcSmoothed)(T*, int);

  public:
    /**
     * Create a new smoothed value
     * @param numSamples The number of samples to store
     * @param calcAvg Callback function to be used in calculating the average
    */
    SmoothedValue(int numSamples, T (*calcSmoothed)(T*, int)){
      this->numSamples = numSamples;
      this->calcSmoothed = calcSmoothed;
      samples = new T[numSamples];
      lastIdx = -1;
    }

    /**
     * Add a new sample
    */
    void addSample(T sample){
      lastIdx ++;
      if(lastIdx >= numSamples) lastIdx = 0;
      samples[lastIdx] = sample;
    }
    
    /**
     * Get the value of the last sample
    */
    T getLast(){
      return samples[lastIdx];
    }

    /**
     * Get smoothed value calculated from samples
    */
    T getSmoothed(){
      return calcSmoothed(samples, numSamples);
    }
};

/**
 * Functions used for static smothing functions
*/
namespace SmoothFunctions {
  
  /**
   * Calculate smoothed by averaging ints
  */
  int smoothInt(int *samples, int count);

  /**
   * Calculate smoothed by averaging floats
  */
  float smoothFloat(float *samples, int count);

  /**
   * Calculate smoothed by averaging booleans
  */
  bool smoothBool(bool *samples, int count);
};

#endif