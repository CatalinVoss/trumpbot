#ifndef line_sensor_hpp
#define line_sensor_hpp

const static int kLineSensorBufferSize = 10; // size of the moving avg filter

/** 
 * Catalin's hardocded 3-part line sensor with moging avg filter
 * Could have implemented this as 2d array, but don't have enough pins to justify that level of flexibility...
 */
class line_sensor {
  public:
    //! Constructor
    line_sensor(int ir1, int ir2, int ir3);

    //! Reads current sensor values
    void read();

    //! Returns true iff the sensor is currently presumed to be on a line
    bool is_line(int pin);

  private:
    int _pin1;
    int _pin2;
    int _pin3;

    int _readings1[kLineSensorBufferSize];
    int _readings2[kLineSensorBufferSize];
    int _readings3[kLineSensorBufferSize];
    int _idx;
};

#endif // line_sensor_hpp