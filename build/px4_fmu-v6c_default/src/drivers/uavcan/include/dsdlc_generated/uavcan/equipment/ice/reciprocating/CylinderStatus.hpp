/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/ice/reciprocating/CylinderStatus.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_ICE_RECIPROCATING_CYLINDERSTATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_ICE_RECIPROCATING_CYLINDERSTATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Cylinder state information.
# This is a nested data type.
#
# All unknown parameters should be set to NaN.
#

#
# Cylinder ignition timing.
# Units: angular degrees of the crankshaft.
#
float16 ignition_timing_deg

#
# Fuel injection time.
# Units: millisecond.
#
float16 injection_time_ms

#
# Cylinder head temperature (CHT).
# Units: kelvin.
#
float16 cylinder_head_temperature

#
# Exhaust gas temperature (EGT).
# Set to NaN if this cylinder is not equipped with an EGT sensor.
# Set this field to the same value for all cylinders if there is a single shared EGT sensor.
# Units: kelvin.
#
float16 exhaust_gas_temperature

#
# Estimated lambda coefficient.
# This parameter is mostly useful for monitoring and tuning purposes.
# Unit: dimensionless ratio
#
float16 lambda_coefficient
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.ice.reciprocating.CylinderStatus
saturated float16 ignition_timing_deg
saturated float16 injection_time_ms
saturated float16 cylinder_head_temperature
saturated float16 exhaust_gas_temperature
saturated float16 lambda_coefficient
******************************************************************************/

#undef ignition_timing_deg
#undef injection_time_ms
#undef cylinder_head_temperature
#undef exhaust_gas_temperature
#undef lambda_coefficient

namespace uavcan
{
namespace equipment
{
namespace ice
{
namespace reciprocating
{

template <int _tmpl>
struct UAVCAN_EXPORT CylinderStatus_
{
    typedef const CylinderStatus_<_tmpl>& ParameterType;
    typedef CylinderStatus_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > ignition_timing_deg;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > injection_time_ms;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > cylinder_head_temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > exhaust_gas_temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > lambda_coefficient;
    };

    enum
    {
        MinBitLen
            = FieldTypes::ignition_timing_deg::MinBitLen
            + FieldTypes::injection_time_ms::MinBitLen
            + FieldTypes::cylinder_head_temperature::MinBitLen
            + FieldTypes::exhaust_gas_temperature::MinBitLen
            + FieldTypes::lambda_coefficient::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::ignition_timing_deg::MaxBitLen
            + FieldTypes::injection_time_ms::MaxBitLen
            + FieldTypes::cylinder_head_temperature::MaxBitLen
            + FieldTypes::exhaust_gas_temperature::MaxBitLen
            + FieldTypes::lambda_coefficient::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::ignition_timing_deg >::Type ignition_timing_deg;
    typename ::uavcan::StorageType< typename FieldTypes::injection_time_ms >::Type injection_time_ms;
    typename ::uavcan::StorageType< typename FieldTypes::cylinder_head_temperature >::Type cylinder_head_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::exhaust_gas_temperature >::Type exhaust_gas_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::lambda_coefficient >::Type lambda_coefficient;

    CylinderStatus_()
        : ignition_timing_deg()
        , injection_time_ms()
        , cylinder_head_temperature()
        , exhaust_gas_temperature()
        , lambda_coefficient()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<80 == MaxBitLen>::check();
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
        return "uavcan.equipment.ice.reciprocating.CylinderStatus";
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
bool CylinderStatus_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        ignition_timing_deg == rhs.ignition_timing_deg &&
        injection_time_ms == rhs.injection_time_ms &&
        cylinder_head_temperature == rhs.cylinder_head_temperature &&
        exhaust_gas_temperature == rhs.exhaust_gas_temperature &&
        lambda_coefficient == rhs.lambda_coefficient;
}

template <int _tmpl>
bool CylinderStatus_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(ignition_timing_deg, rhs.ignition_timing_deg) &&
        ::uavcan::areClose(injection_time_ms, rhs.injection_time_ms) &&
        ::uavcan::areClose(cylinder_head_temperature, rhs.cylinder_head_temperature) &&
        ::uavcan::areClose(exhaust_gas_temperature, rhs.exhaust_gas_temperature) &&
        ::uavcan::areClose(lambda_coefficient, rhs.lambda_coefficient);
}

template <int _tmpl>
int CylinderStatus_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::ignition_timing_deg::encode(self.ignition_timing_deg, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::injection_time_ms::encode(self.injection_time_ms, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::cylinder_head_temperature::encode(self.cylinder_head_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::exhaust_gas_temperature::encode(self.exhaust_gas_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::lambda_coefficient::encode(self.lambda_coefficient, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int CylinderStatus_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::ignition_timing_deg::decode(self.ignition_timing_deg, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::injection_time_ms::decode(self.injection_time_ms, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::cylinder_head_temperature::decode(self.cylinder_head_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::exhaust_gas_temperature::decode(self.exhaust_gas_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::lambda_coefficient::decode(self.lambda_coefficient, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature CylinderStatus_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xD68AC83A89D5B36BULL);

    FieldTypes::ignition_timing_deg::extendDataTypeSignature(signature);
    FieldTypes::injection_time_ms::extendDataTypeSignature(signature);
    FieldTypes::cylinder_head_temperature::extendDataTypeSignature(signature);
    FieldTypes::exhaust_gas_temperature::extendDataTypeSignature(signature);
    FieldTypes::lambda_coefficient::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef CylinderStatus_<0> CylinderStatus;

// No default registration

} // Namespace reciprocating
} // Namespace ice
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::ice::reciprocating::CylinderStatus::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus >::stream(Stream& s, ::uavcan::equipment::ice::reciprocating::CylinderStatus::ParameterType obj, const int level)
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
    s << "ignition_timing_deg: ";
    YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus::FieldTypes::ignition_timing_deg >::stream(s, obj.ignition_timing_deg, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "injection_time_ms: ";
    YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus::FieldTypes::injection_time_ms >::stream(s, obj.injection_time_ms, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "cylinder_head_temperature: ";
    YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus::FieldTypes::cylinder_head_temperature >::stream(s, obj.cylinder_head_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "exhaust_gas_temperature: ";
    YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus::FieldTypes::exhaust_gas_temperature >::stream(s, obj.exhaust_gas_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "lambda_coefficient: ";
    YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus::FieldTypes::lambda_coefficient >::stream(s, obj.lambda_coefficient, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace ice
{
namespace reciprocating
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::ice::reciprocating::CylinderStatus::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::ice::reciprocating::CylinderStatus >::stream(s, obj, 0);
    return s;
}

} // Namespace reciprocating
} // Namespace ice
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_ICE_RECIPROCATING_CYLINDERSTATUS_HPP_INCLUDED