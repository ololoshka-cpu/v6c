/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/ardupilot/gnss/20003.Status.uavcan
 */

#ifndef ARDUPILOT_GNSS_STATUS_HPP_INCLUDED
#define ARDUPILOT_GNSS_STATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
# Node specific GNSS error codes, primarily available for logging and diagnostics
uint32 error_codes

# GNSS system is self assesd as healthy
bool healthy

# Status is actually a bitmask, due to encoding issues pretend it's just a field, and leave it up to the application do decode it)
uint23 STATUS_LOGGING = 1  # GNSS system is doing any onboard logging
uint23 STATUS_ARMABLE = 2  # GNSS system is in a reasonable state to allow the system to arm
uint23 status
******************************************************************************/

/********************* DSDL signature source definition ***********************
ardupilot.gnss.Status
saturated uint32 error_codes
saturated bool healthy
saturated uint23 status
******************************************************************************/

#undef error_codes
#undef healthy
#undef status
#undef STATUS_LOGGING
#undef STATUS_ARMABLE

namespace ardupilot
{
namespace gnss
{

template <int _tmpl>
struct UAVCAN_EXPORT Status_
{
    typedef const Status_<_tmpl>& ParameterType;
    typedef Status_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 23, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_LOGGING;
        typedef ::uavcan::IntegerSpec< 23, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_ARMABLE;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > error_codes;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > healthy;
        typedef ::uavcan::IntegerSpec< 23, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > status;
    };

    enum
    {
        MinBitLen
            = FieldTypes::error_codes::MinBitLen
            + FieldTypes::healthy::MinBitLen
            + FieldTypes::status::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::error_codes::MaxBitLen
            + FieldTypes::healthy::MaxBitLen
            + FieldTypes::status::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_LOGGING >::Type STATUS_LOGGING; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_ARMABLE >::Type STATUS_ARMABLE; // 2

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::error_codes >::Type error_codes;
    typename ::uavcan::StorageType< typename FieldTypes::healthy >::Type healthy;
    typename ::uavcan::StorageType< typename FieldTypes::status >::Type status;

    Status_()
        : error_codes()
        , healthy()
        , status()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<56 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 20003 };

    static const char* getDataTypeFullName()
    {
        return "ardupilot.gnss.Status";
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
bool Status_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        error_codes == rhs.error_codes &&
        healthy == rhs.healthy &&
        status == rhs.status;
}

template <int _tmpl>
bool Status_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(error_codes, rhs.error_codes) &&
        ::uavcan::areClose(healthy, rhs.healthy) &&
        ::uavcan::areClose(status, rhs.status);
}

template <int _tmpl>
int Status_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error_codes::encode(self.error_codes, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::healthy::encode(self.healthy, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status::encode(self.status, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Status_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error_codes::decode(self.error_codes, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::healthy::decode(self.healthy, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status::decode(self.status, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Status_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xBA3CB4ABBB007F69ULL);

    FieldTypes::error_codes::extendDataTypeSignature(signature);
    FieldTypes::healthy::extendDataTypeSignature(signature);
    FieldTypes::status::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Status_<_tmpl>::ConstantTypes::STATUS_LOGGING >::Type
    Status_<_tmpl>::STATUS_LOGGING = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename Status_<_tmpl>::ConstantTypes::STATUS_ARMABLE >::Type
    Status_<_tmpl>::STATUS_ARMABLE = 2U; // 2

/*
 * Final typedef
 */
typedef Status_<0> Status;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::ardupilot::gnss::Status > _uavcan_gdtr_registrator_Status;

}

} // Namespace gnss
} // Namespace ardupilot

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::ardupilot::gnss::Status >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::ardupilot::gnss::Status::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::ardupilot::gnss::Status >::stream(Stream& s, ::ardupilot::gnss::Status::ParameterType obj, const int level)
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
    s << "error_codes: ";
    YamlStreamer< ::ardupilot::gnss::Status::FieldTypes::error_codes >::stream(s, obj.error_codes, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "healthy: ";
    YamlStreamer< ::ardupilot::gnss::Status::FieldTypes::healthy >::stream(s, obj.healthy, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "status: ";
    YamlStreamer< ::ardupilot::gnss::Status::FieldTypes::status >::stream(s, obj.status, level + 1);
}

}

namespace ardupilot
{
namespace gnss
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::ardupilot::gnss::Status::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::ardupilot::gnss::Status >::stream(s, obj, 0);
    return s;
}

} // Namespace gnss
} // Namespace ardupilot

#endif // ARDUPILOT_GNSS_STATUS_HPP_INCLUDED