// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: thruster_command.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_thruster_5fcommand_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_thruster_5fcommand_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3012000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3012004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_thruster_5fcommand_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_thruster_5fcommand_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_thruster_5fcommand_2eproto;
namespace communication {
class Command;
class CommandDefaultTypeInternal;
extern CommandDefaultTypeInternal _Command_default_instance_;
}  // namespace communication
PROTOBUF_NAMESPACE_OPEN
template<> ::communication::Command* Arena::CreateMaybeMessage<::communication::Command>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace communication {

// ===================================================================

class Command PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:communication.Command) */ {
 public:
  inline Command() : Command(nullptr) {};
  virtual ~Command();

  Command(const Command& from);
  Command(Command&& from) noexcept
    : Command() {
    *this = ::std::move(from);
  }

  inline Command& operator=(const Command& from) {
    CopyFrom(from);
    return *this;
  }
  inline Command& operator=(Command&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Command& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Command* internal_default_instance() {
    return reinterpret_cast<const Command*>(
               &_Command_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Command& a, Command& b) {
    a.Swap(&b);
  }
  inline void Swap(Command* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Command* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Command* New() const final {
    return CreateMaybeMessage<Command>(nullptr);
  }

  Command* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Command>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Command& from);
  void MergeFrom(const Command& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Command* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "communication.Command";
  }
  protected:
  explicit Command(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_thruster_5fcommand_2eproto);
    return ::descriptor_table_thruster_5fcommand_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kThrustFieldNumber = 1,
    kEmergencyStopFieldNumber = 2,
  };
  // double thrust = 1;
  void clear_thrust();
  double thrust() const;
  void set_thrust(double value);
  private:
  double _internal_thrust() const;
  void _internal_set_thrust(double value);
  public:

  // bool emergency_stop = 2;
  void clear_emergency_stop();
  bool emergency_stop() const;
  void set_emergency_stop(bool value);
  private:
  bool _internal_emergency_stop() const;
  void _internal_set_emergency_stop(bool value);
  public:

  // @@protoc_insertion_point(class_scope:communication.Command)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  double thrust_;
  bool emergency_stop_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_thruster_5fcommand_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Command

// double thrust = 1;
inline void Command::clear_thrust() {
  thrust_ = 0;
}
inline double Command::_internal_thrust() const {
  return thrust_;
}
inline double Command::thrust() const {
  // @@protoc_insertion_point(field_get:communication.Command.thrust)
  return _internal_thrust();
}
inline void Command::_internal_set_thrust(double value) {
  
  thrust_ = value;
}
inline void Command::set_thrust(double value) {
  _internal_set_thrust(value);
  // @@protoc_insertion_point(field_set:communication.Command.thrust)
}

// bool emergency_stop = 2;
inline void Command::clear_emergency_stop() {
  emergency_stop_ = false;
}
inline bool Command::_internal_emergency_stop() const {
  return emergency_stop_;
}
inline bool Command::emergency_stop() const {
  // @@protoc_insertion_point(field_get:communication.Command.emergency_stop)
  return _internal_emergency_stop();
}
inline void Command::_internal_set_emergency_stop(bool value) {
  
  emergency_stop_ = value;
}
inline void Command::set_emergency_stop(bool value) {
  _internal_set_emergency_stop(value);
  // @@protoc_insertion_point(field_set:communication.Command.emergency_stop)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace communication

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_thruster_5fcommand_2eproto
