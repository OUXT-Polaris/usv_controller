// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: thruster_command.proto

#include "usv_controller/thruster_command.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace communication {
class CommandDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Command> _instance;
} _Command_default_instance_;
}  // namespace communication
static void InitDefaultsscc_info_Command_thruster_5fcommand_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::communication::_Command_default_instance_;
    new (ptr) ::communication::Command();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::communication::Command::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Command_thruster_5fcommand_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Command_thruster_5fcommand_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_thruster_5fcommand_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_thruster_5fcommand_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_thruster_5fcommand_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_thruster_5fcommand_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::communication::Command, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::communication::Command, thrust_),
  PROTOBUF_FIELD_OFFSET(::communication::Command, emergency_stop_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::communication::Command)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::communication::_Command_default_instance_),
};

const char descriptor_table_protodef_thruster_5fcommand_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\026thruster_command.proto\022\rcommunication\""
  "1\n\007Command\022\016\n\006thrust\030\001 \001(\001\022\026\n\016emergency_"
  "stop\030\002 \001(\010b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_thruster_5fcommand_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_thruster_5fcommand_2eproto_sccs[1] = {
  &scc_info_Command_thruster_5fcommand_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_thruster_5fcommand_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_thruster_5fcommand_2eproto = {
  false, false, descriptor_table_protodef_thruster_5fcommand_2eproto, "thruster_command.proto", 98,
  &descriptor_table_thruster_5fcommand_2eproto_once, descriptor_table_thruster_5fcommand_2eproto_sccs, descriptor_table_thruster_5fcommand_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_thruster_5fcommand_2eproto::offsets,
  file_level_metadata_thruster_5fcommand_2eproto, 1, file_level_enum_descriptors_thruster_5fcommand_2eproto, file_level_service_descriptors_thruster_5fcommand_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_thruster_5fcommand_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_thruster_5fcommand_2eproto)), true);
namespace communication {

// ===================================================================

void Command::InitAsDefaultInstance() {
}
class Command::_Internal {
 public:
};

Command::Command(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:communication.Command)
}
Command::Command(const Command& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&thrust_, &from.thrust_,
    static_cast<size_t>(reinterpret_cast<char*>(&emergency_stop_) -
    reinterpret_cast<char*>(&thrust_)) + sizeof(emergency_stop_));
  // @@protoc_insertion_point(copy_constructor:communication.Command)
}

void Command::SharedCtor() {
  ::memset(&thrust_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&emergency_stop_) -
      reinterpret_cast<char*>(&thrust_)) + sizeof(emergency_stop_));
}

Command::~Command() {
  // @@protoc_insertion_point(destructor:communication.Command)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void Command::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void Command::ArenaDtor(void* object) {
  Command* _this = reinterpret_cast< Command* >(object);
  (void)_this;
}
void Command::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Command::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Command& Command::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Command_thruster_5fcommand_2eproto.base);
  return *internal_default_instance();
}


void Command::Clear() {
// @@protoc_insertion_point(message_clear_start:communication.Command)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&thrust_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&emergency_stop_) -
      reinterpret_cast<char*>(&thrust_)) + sizeof(emergency_stop_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Command::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // double thrust = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          thrust_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // bool emergency_stop = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          emergency_stop_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Command::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:communication.Command)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double thrust = 1;
  if (!(this->thrust() <= 0 && this->thrust() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_thrust(), target);
  }

  // bool emergency_stop = 2;
  if (this->emergency_stop() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2, this->_internal_emergency_stop(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:communication.Command)
  return target;
}

size_t Command::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:communication.Command)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double thrust = 1;
  if (!(this->thrust() <= 0 && this->thrust() >= 0)) {
    total_size += 1 + 8;
  }

  // bool emergency_stop = 2;
  if (this->emergency_stop() != 0) {
    total_size += 1 + 1;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Command::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:communication.Command)
  GOOGLE_DCHECK_NE(&from, this);
  const Command* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Command>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:communication.Command)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:communication.Command)
    MergeFrom(*source);
  }
}

void Command::MergeFrom(const Command& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:communication.Command)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!(from.thrust() <= 0 && from.thrust() >= 0)) {
    _internal_set_thrust(from._internal_thrust());
  }
  if (from.emergency_stop() != 0) {
    _internal_set_emergency_stop(from._internal_emergency_stop());
  }
}

void Command::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:communication.Command)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Command::CopyFrom(const Command& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:communication.Command)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Command::IsInitialized() const {
  return true;
}

void Command::InternalSwap(Command* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Command, emergency_stop_)
      + sizeof(Command::emergency_stop_)
      - PROTOBUF_FIELD_OFFSET(Command, thrust_)>(
          reinterpret_cast<char*>(&thrust_),
          reinterpret_cast<char*>(&other->thrust_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Command::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace communication
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::communication::Command* Arena::CreateMaybeMessage< ::communication::Command >(Arena* arena) {
  return Arena::CreateMessageInternal< ::communication::Command >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
