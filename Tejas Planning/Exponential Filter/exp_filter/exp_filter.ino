#include "Filter.h"
 
// the <float> makes a filter for float numbers
// 20 is the weight (20 => 20%)
// 0 is the initial value of the filter
ExponentialFilter<float> FilteredTemperature(20, 0);
 
void loop() {
  float RawTemperature = MeasureTemperature();
  FilteredTemperature.Filter(RawTemperature);
  float SmoothTemperature = FilteredTemperature.Current();
}
