// Templatized implementation of a data stream object

#ifndef DATA_STREAM_H
#define DATA_STREAM_H

#include <Arduino.h>


void close(const char* fname)
{
    Serial.print("_data_stream_:_close_:");
    Serial.println(fname);
}

// Templatized version of data stream
template <typename T>
class DataStream
{
public:
    DataStream(const char* name, int num_data_points);

    const char* get_name();
    int get_num_data_points();
    T* get_data();

    // Add a new data point to the data stream
    // This will append until the data stream is full, then it will return false and not add the data point
    bool add_data_point(T data_point);

    // Delete the existing data
    void reset();

    // Flush data stream to serial
    // Allow user to specify a postfix to append to the name of the data stream
    void output_data_stream(const char* postfix);
private:

    const char* name;
    int num_data_points;
    T* data;
    int cur_data_point = 0;
};

template <typename T>
DataStream<T>::DataStream(const char* name, int num_data_points)
{
    this->name = name;
    this->num_data_points = num_data_points;
    this->data = new T[num_data_points];
}

template <typename T>
const char* DataStream<T>::get_name()
{
    return name;
}

template <typename T>
int DataStream<T>::get_num_data_points()
{
    return num_data_points;
}

template <typename T>
T* DataStream<T>::get_data()
{
    return data;
}

template <typename T>
bool DataStream<T>::add_data_point(T data_point)
{
    if(cur_data_point < num_data_points)
    {
        // Append data and increment counter
        data[cur_data_point] = data_point;
        cur_data_point++;

        return true;
    }

    return false;
}

template <typename T>
void DataStream<T>::reset()
{
    cur_data_point = 0;
}


// Templatized function to output a data stream to a serial port
template <typename T>
void DataStream<T>::output_data_stream(const char* postfix)
{
    // Print the name of the data stream
    Serial.print("_data_stream_:");
    Serial.print(get_name());
    Serial.print("_");
    Serial.print(postfix);
    Serial.print(":");

    // Print the data
    for(int i = 0; i < get_num_data_points(); i++)
    {
        Serial.print(data[i], 10);
        Serial.print(",");
    }

    // Print a newline
    Serial.println();
}

#endif
