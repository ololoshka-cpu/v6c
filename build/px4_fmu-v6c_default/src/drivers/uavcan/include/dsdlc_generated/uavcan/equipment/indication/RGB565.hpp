/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/indication/RGB565.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_INDICATION_RGB565_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_INDICATION_RGB565_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested type.
# RGB color in the standard 5-6-5 16-bit palette.
# Monocolor lights should interpret this as brightness setpoint: from zero (0, 0, 0) to full brightness (31, 63, 31).
#

uint5 red
uint6 green
uint5 blue
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.indication.RGB565
saturated uint5 red
saturated uint6 green
saturated uint5 blue
******************************************************************************/

#undef red
#undef green
#undef blue

namespace uavcan
{
namespace equipment
{
namespace indication
{

template <int _tmpl>
struct UAVCAN_EXPORT RGB565_
{
    typedef const RGB565_<_tmpl>& ParameterType;
    typedef RGB565_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 5, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > red;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > green;
        typedef ::uavcan::IntegerSpec< 5, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > blue;
    };

    enum
    {
        MinBitLen
            = FieldTypes::red::MinBitLen
            + FieldTypes::green::MinBitLen
            + FieldTypes::blue::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::red::MaxBitLen
            + FieldTypes::green::MaxBitLen
            + FieldTypes::blue::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::red >::Type red;
    typename ::uavcan::StorageType< typename FieldTypes::green >::Type green;
    typename ::uavcan::StorageType< typename FieldTypes::blue >::Type blue;

    RGB565_()
        : red()
        , green()
        , blue()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<16 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.indication.RGB565";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RGB565_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        red == rhs.red &&
        green == rhs.green &&
        blue == rhs.blue;
}

template <int _tmpl>
bool RGB565_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(red, rhs.red) &&
        ::uavcan::areClose(green, rhs.green) &&
        ::uavcan::areClose(blue, rhs.blue);
}

template <int _tmpl>
int RGB565_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::red::encode(self.red, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::green::encode(self.green, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::blue::encode(self.blue, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RGB565_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::red::decode(self.red, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::green::decode(self.green, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::blue::decode(self.blue, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RGB565_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x58A7CEF41951EC34ULL);

    FieldTypes::red::extendDataTypeSignature(signature);
    FieldTypes::green::extendDataTypeSignature(signature);
    FieldTypes::blue::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef RGB565_<0> RGB565;

// No default registration

} // Namespace indication
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::indication::RGB565 >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::indication::RGB565::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::indication::RGB565 >::stream(Stream& s, ::uavcan::equipment::indication::RGB565::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "red: ";
    YamlStreamer< ::uavcan::equipment::indication::RGB565::FieldTypes::red >::stream(s, obj.red, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "green: ";
    YamlStreamer< ::uavcan::equipment::indication::RGB565::FieldTypes::green >::stream(s, obj.green, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "blue: ";
    YamlStreamer< ::uavcan::equipment::indication::RGB565::FieldTypes::blue >::stream(s, obj.blue, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace indication
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::indication::RGB565::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::indication::RGB565 >::stream(s, obj, 0);
    return s;
}

} // Namespace indication
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_INDICATION_RGB565_HPP_INCLUDED