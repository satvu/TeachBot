// Generated by gencpp from file button_box/ButtonInfoResponse.msg
// DO NOT EDIT!


#ifndef BUTTON_BOX_MESSAGE_BUTTONINFORESPONSE_H
#define BUTTON_BOX_MESSAGE_BUTTONINFORESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace button_box
{
template <class ContainerAllocator>
struct ButtonInfoResponse_
{
  typedef ButtonInfoResponse_<ContainerAllocator> Type;

  ButtonInfoResponse_()
    : response()  {
    }
  ButtonInfoResponse_(const ContainerAllocator& _alloc)
    : response(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _response_type;
  _response_type response;





  typedef boost::shared_ptr< ::button_box::ButtonInfoResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::button_box::ButtonInfoResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ButtonInfoResponse_

typedef ::button_box::ButtonInfoResponse_<std::allocator<void> > ButtonInfoResponse;

typedef boost::shared_ptr< ::button_box::ButtonInfoResponse > ButtonInfoResponsePtr;
typedef boost::shared_ptr< ::button_box::ButtonInfoResponse const> ButtonInfoResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::button_box::ButtonInfoResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::button_box::ButtonInfoResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace button_box

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::button_box::ButtonInfoResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::button_box::ButtonInfoResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::button_box::ButtonInfoResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6de314e2dc76fbff2b6244a6ad70b68d";
  }

  static const char* value(const ::button_box::ButtonInfoResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6de314e2dc76fbffULL;
  static const uint64_t static_value2 = 0x2b6244a6ad70b68dULL;
};

template<class ContainerAllocator>
struct DataType< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "button_box/ButtonInfoResponse";
  }

  static const char* value(const ::button_box::ButtonInfoResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string response\n\
\n\
";
  }

  static const char* value(const ::button_box::ButtonInfoResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.response);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ButtonInfoResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::button_box::ButtonInfoResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::button_box::ButtonInfoResponse_<ContainerAllocator>& v)
  {
    s << indent << "response: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.response);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BUTTON_BOX_MESSAGE_BUTTONINFORESPONSE_H