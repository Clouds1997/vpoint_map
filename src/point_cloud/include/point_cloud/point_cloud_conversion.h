#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include<iostream>

namespace point_cloud_conversion
{ 

    static void convertPointCloud2ToPointCloud(const sensor_msgs::PointCloud2 &input, sensor_msgs::PointCloud &output)
    { 
        output.header = input.header;
        output.points.resize (input.width * input.height);
        output.channels.resize (input.fields.size () - 3);
        // Get the x/y/z field offsets
        int x_idx = sensor_msgs::getPointCloud2FieldIndex (input, "x");
        int y_idx = sensor_msgs::getPointCloud2FieldIndex (input, "y");
        int z_idx = sensor_msgs::getPointCloud2FieldIndex (input, "z");
        if (x_idx == -1 || y_idx == -1 || z_idx == -1)
        {
            std::cerr << "x/y/z coordinates not found! Cannot convert to sensor_msgs::PointCloud!" << std::endl;
        }
        int x_offset = input.fields[x_idx].offset;
        int y_offset = input.fields[y_idx].offset;
        int z_offset = input.fields[z_idx].offset;
        uint8_t x_datatype = input.fields[x_idx].datatype;
        uint8_t y_datatype = input.fields[y_idx].datatype;
        uint8_t z_datatype = input.fields[z_idx].datatype;
        
        // Convert the fields to channels
        int cur_c = 0;
        for (size_t d = 0; d < input.fields.size (); ++d)
        {
            if ((int)input.fields[d].offset == x_offset || (int)input.fields[d].offset == y_offset || (int)input.fields[d].offset == z_offset)
            continue;
            output.channels[cur_c].name = input.fields[d].name;
            output.channels[cur_c].values.resize (output.points.size ());
            cur_c++;
        }

        // Copy the data points
        for (size_t cp = 0; cp < output.points.size (); ++cp)
        {
            output.points[cp].x =-0.0/0.0 ;
            output.points[cp].y =-0.0/0.0 ;
            output.points[cp].z =-0.0/0.0 ;
            // Copy x/y/z
            if(cp > 460800 && sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + z_offset], z_datatype) <-0.55 
            && sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + x_offset], x_datatype) <3.5)
            { 
            output.points[cp].x = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + x_offset], x_datatype);
            output.points[cp].y = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + y_offset], y_datatype);
            output.points[cp].z = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + z_offset], z_datatype);
            // std::cout<<output.points[cp].z<<endl;
            // output.points[cp].z =-0.0/0.0 ;
            }
            // output.points[cp].x = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + x_offset], x_datatype);
            // output.points[cp].y = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + y_offset], y_datatype);
            // // output.points[cp].z = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + z_offset], z_datatype);
            // output.points[cp].z =-0.0/0.0 ;
            // // Copy the rest of the data
            int cur_c = 0;
            for (size_t d = 0; d < input.fields.size (); ++d)
            {
            if ((int)input.fields[d].offset == x_offset || (int)input.fields[d].offset == y_offset || (int)input.fields[d].offset == z_offset)
                continue;
            output.channels[cur_c++].values[cp] = sensor_msgs::readPointCloud2BufferValue<float>(&input.data[cp * input.point_step + input.fields[d].offset], input.fields[d].datatype);
            }
        }
    }

};