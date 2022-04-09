
#ifndef H_6B9572DA_A64B_49E6_B234_051480991C89
#define H_6B9572DA_A64B_49E6_B234_051480991C89

#ifndef __cplusplus
#	error "It's not going to compile without a C++ compiler..."
#endif

#if	  defined(BACKWARD_CXX11)
#elif defined(BACKWARD_CXX98)
#else
#	if __cplusplus >= 201103L
#		define BACKWARD_CXX11
#		define BACKWARD_ATLEAST_CXX11
#		define BACKWARD_ATLEAST_CXX98
#	else
#		define BACKWARD_CXX98
#		define BACKWARD_ATLEAST_CXX98
#	endif
#endif

#if   defined(BACKWARD_SYSTEM_LINUX)
#elif defined(BACKWARD_SYSTEM_UNKNOWN)
#else
#	if defined(__linux)
#		define BACKWARD_SYSTEM_LINUX
#	else
#		define BACKWARD_SYSTEM_UNKNOWN
#	endif
#endif

#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <string>
#include <new>
#include <iomanip>
#include <vector>

#if defined(BACKWARD_SYSTEM_LINUX)

#	if   BACKWARD_HAS_UNWIND == 1
#	elif BACKWARD_HAS_BACKTRACE == 1
#	else
#		undef  BACKWARD_HAS_UNWIND
#		define BACKWARD_HAS_UNWIND 1
#		undef  BACKWARD_HAS_BACKTRACE
#		define BACKWARD_HAS_BACKTRACE 0
#	endif

#	if   BACKWARD_HAS_DW == 1
#	elif BACKWARD_HAS_BFD == 1
#	elif BACKWARD_HAS_BACKTRACE_SYMBOL == 1
#	else
#		undef  BACKWARD_HAS_DW
#		define BACKWARD_HAS_DW 0
#		undef  BACKWARD_HAS_BFD
#		define BACKWARD_HAS_BFD 0
#		undef  BACKWARD_HAS_BACKTRACE_SYMBOL
#		define BACKWARD_HAS_BACKTRACE_SYMBOL 1
#	endif


#	if BACKWARD_HAS_UNWIND == 1

#		include <unwind.h>

#ifdef __CLANG_UNWIND_H

#		include <inttypes.h>
extern "C" uintptr_t _Unwind_GetIPInfo(_Unwind_Context*, int*);
#endif

#	endif

#	include <cxxabi.h>
#	include <fcntl.h>
#	include <link.h>
#	include <sys/stat.h>
#	include <syscall.h>
#	include <unistd.h>
#	include <signal.h>

#	if BACKWARD_HAS_BFD == 1
//              NOTE: defining PACKAGE{,_VERSION} is required before including
//                    bfd.h on some platforms, see also:
//                    https://sourceware.org/bugzilla/show_bug.cgi?id=14243
#               ifndef PACKAGE
#                       define PACKAGE
#               endif
#               ifndef PACKAGE_VERSION
#                       define PACKAGE_VERSION
#               endif
#		include <bfd.h>
#		ifndef _GNU_SOURCE
#			define _GNU_SOURCE
#			include <dlfcn.h>
#			undef _GNU_SOURCE
#		else
#			include <dlfcn.h>
#		endif
#	endif

#	if BACKWARD_HAS_DW == 1
#		include <elfutils/libdw.h>
#		include <elfutils/libdwfl.h>
#		include <dwarf.h>
#	endif

#	if (BACKWARD_HAS_BACKTRACE == 1) || (BACKWARD_HAS_BACKTRACE_SYMBOL == 1)
		
#		include <execinfo.h>
#	endif

#endif 

#ifdef BACKWARD_ATLEAST_CXX11
#	include <unordered_map>
#	include <utility> 
	namespace backward {
	namespace details {
		template <typename K, typename V>
		struct hashtable {
			typedef std::unordered_map<K, V> type;
		};
		using std::move;
	} 
	} 
#else 
#	include <map>
	namespace backward {
	namespace details {
		template <typename K, typename V>
		struct hashtable {
			typedef std::map<K, V> type;
		};
		template <typename T>
			const T& move(const T& v) { return v; }
		template <typename T>
			T& move(T& v) { return v; }
	} 
	} 
#endif 

namespace backward {

namespace system_tag {
	struct linux_tag; 
	struct unknown_tag;

#if   defined(BACKWARD_SYSTEM_LINUX)
	typedef linux_tag current_tag;
#elif defined(BACKWARD_SYSTEM_UNKNOWN)
	typedef unknown_tag current_tag;
#else
#	error "May I please get my system defines?"
#endif
} 


namespace trace_resolver_tag {
#ifdef BACKWARD_SYSTEM_LINUX
	struct libdw;
	struct libbfd;
	struct backtrace_symbol;

#	if   BACKWARD_HAS_DW == 1
	typedef libdw current;
#	elif BACKWARD_HAS_BFD == 1
	typedef libbfd current;
#	elif BACKWARD_HAS_BACKTRACE_SYMBOL == 1
	typedef backtrace_symbol current;
#	else
#		error "You shall not pass, until you know what you want."
#	endif
#endif 
} 


namespace details {

template <typename T>
	struct rm_ptr { typedef T type; };

template <typename T>
	struct rm_ptr<T*> { typedef T type; };

template <typename T>
	struct rm_ptr<const T*> { typedef const T type; };

template <typename R, typename T, R (*F)(T)>
struct deleter {
	template <typename U>
		void operator()(U& ptr) const {
			(*F)(ptr);
		}
};

template <typename T>
struct default_delete {
	void operator()(T& ptr) const {
		delete ptr;
	}
};

template <typename T, typename Deleter = deleter<void, void*, &::free> >
class handle {
	struct dummy;
	T    _val;
	bool _empty;

#ifdef BACKWARD_ATLEAST_CXX11
	handle(const handle&) = delete;
	handle& operator=(const handle&) = delete;
#endif

public:
	~handle() {
		if (!_empty) {
			Deleter()(_val);
		}
	}

	explicit handle(): _val(), _empty(true) {}
	explicit handle(T val): _val(val), _empty(false) {}

#ifdef BACKWARD_ATLEAST_CXX11
	handle(handle&& from): _empty(true) {
		swap(from);
	}
	handle& operator=(handle&& from) {
		swap(from); return *this;
	}
#else
	explicit handle(const handle& from): _empty(true) {
		// some sort of poor man's move semantic.
		swap(const_cast<handle&>(from));
	}
	handle& operator=(const handle& from) {
		// some sort of poor man's move semantic.
		swap(const_cast<handle&>(from)); return *this;
	}
#endif

	void reset(T new_val) {
		handle tmp(new_val);
		swap(tmp);
	}
	operator const dummy*() const {
		if (_empty) {
			return 0;
		}
		return reinterpret_cast<const dummy*>(_val);
	}
	T get() {
		return _val;
	}
	T release() {
		_empty = true;
		return _val;
	}
	void swap(handle& b) {
		using std::swap;
		swap(b._val, _val); 
		swap(b._empty, _empty);
	}

	T operator->() { return _val; }
	const T operator->() const { return _val; }

	typedef typename rm_ptr<T>::type& ref_t;
	typedef const typename rm_ptr<T>::type& const_ref_t;
	ref_t operator*() { return *_val; }
	const_ref_t operator*() const { return *_val; }
	ref_t operator[](size_t idx) { return _val[idx]; }

	
	T* operator&() {
		_empty = false;
		return &_val;
	}
};


template <typename TAG>
struct demangler_impl {
	static std::string demangle(const char* funcname) {
		return funcname;
	}
};

#ifdef BACKWARD_SYSTEM_LINUX

template <>
struct demangler_impl<system_tag::current_tag> {
	demangler_impl(): _demangle_buffer_length(0) {}

	std::string demangle(const char* funcname) {
		using namespace details;
		_demangle_buffer.reset(
				abi::__cxa_demangle(funcname, _demangle_buffer.release(),
					&_demangle_buffer_length, 0)
				);
		if (_demangle_buffer) {
			return _demangle_buffer.get();
		}
		return funcname;
	}

private:
	details::handle<char*> _demangle_buffer;
	size_t                 _demangle_buffer_length;
};

#endif 

struct demangler:
	public demangler_impl<system_tag::current_tag> {};

} 

/*************** A TRACE ***************/

struct Trace {
	void*    addr;
	unsigned idx;

	Trace():
		addr(0), idx(0) {}

	explicit Trace(void* addr, size_t idx):
		addr(addr), idx(idx) {}
};

struct ResolvedTrace: public Trace {

	struct SourceLoc {
		std::string function;
		std::string filename;
		unsigned    line;
		unsigned    col;

		SourceLoc(): line(0), col(0) {}

		bool operator==(const SourceLoc& b) const {
			return function == b.function
				&& filename == b.filename
				&& line == b.line
				&& col == b.col;
		}

		bool operator!=(const SourceLoc& b) const {
			return !(*this == b);
		}
	};

	
	std::string                    object_filename;

	
	std::string                    object_function;

	SourceLoc                      source;

	typedef std::vector<SourceLoc> source_locs_t;
	source_locs_t                  inliners;

	ResolvedTrace():
		Trace() {}
	ResolvedTrace(const Trace& mini_trace):
		Trace(mini_trace) {}
};

/*************** STACK TRACE ***************/


template <typename TAG>
class StackTraceImpl {
public:
	size_t size() const { return 0; }
	Trace operator[](size_t) { return Trace(); }
	size_t load_here(size_t=0) { return 0; }
	size_t load_from(void*, size_t=0) { return 0; }
	unsigned thread_id() const { return 0; }
};

#ifdef BACKWARD_SYSTEM_LINUX

class StackTraceLinuxImplBase {
public:
	StackTraceLinuxImplBase(): _thread_id(0), _skip(0) {}

	unsigned thread_id() const {
		return _thread_id;
	}

protected:
	void load_thread_info() {
		_thread_id = syscall(SYS_gettid);
		if (_thread_id == (size_t) getpid()) {
			
			_thread_id = 0;
		}
	}

	void skip_n_firsts(size_t n) { _skip = n; }
	size_t skip_n_firsts() const { return _skip; }

private:
	size_t _thread_id;
	size_t _skip;
};

class StackTraceLinuxImplHolder: public StackTraceLinuxImplBase {
public:
	size_t size() const {
		return _stacktrace.size() ? _stacktrace.size() - skip_n_firsts() : 0;
	}
	Trace operator[](size_t idx) {
		if (idx >= size()) {
			return Trace();
		}
		return Trace(_stacktrace[idx + skip_n_firsts()], idx);
	}
	void** begin() {
		if (size()) {
			return &_stacktrace[skip_n_firsts()];
		}
		return 0;
	}

protected:
	std::vector<void*> _stacktrace;
};


#if BACKWARD_HAS_UNWIND == 1

namespace details {

template <typename F>
class Unwinder {
public:
	size_t operator()(F& f, size_t depth) {
		_f = &f;
		_index = -1;
		_depth = depth;
		_Unwind_Backtrace(&this->backtrace_trampoline, this);
		return _index;
	}

private:
	F*      _f;
	ssize_t _index;
	size_t  _depth;

	static _Unwind_Reason_Code backtrace_trampoline(
			_Unwind_Context* ctx, void *self) {
		return ((Unwinder*)self)->backtrace(ctx);
	}

	_Unwind_Reason_Code backtrace(_Unwind_Context* ctx) {
		if (_index >= 0 && static_cast<size_t>(_index) >= _depth)
			return _URC_END_OF_STACK;

		int ip_before_instruction = 0;
		uintptr_t ip = _Unwind_GetIPInfo(ctx, &ip_before_instruction);

		if (!ip_before_instruction) {
			ip -= 1;
		}

		if (_index >= 0) { 
			(*_f)(_index, (void*)ip);
		}
		_index += 1;
		return _URC_NO_REASON;
	}
};

template <typename F>
size_t unwind(F f, size_t depth) {
	Unwinder<F> unwinder;
	return unwinder(f, depth);
}

}


template <>
class StackTraceImpl<system_tag::linux_tag>: public StackTraceLinuxImplHolder {
public:
	__attribute__ ((noinline)) 
	size_t load_here(size_t depth=32) {
		load_thread_info();
		if (depth == 0) {
			return 0;
		}
		_stacktrace.resize(depth);
		size_t trace_cnt = details::unwind(callback(*this), depth);
		_stacktrace.resize(trace_cnt);
		skip_n_firsts(0);
		return size();
	}
	size_t load_from(void* addr, size_t depth=32) {
		load_here(depth + 8);

		for (size_t i = 0; i < _stacktrace.size(); ++i) {
			if (_stacktrace[i] == addr) {
				skip_n_firsts(i);
				break;
			}
		}

		_stacktrace.resize(std::min(_stacktrace.size(),
					skip_n_firsts() + depth));
		return size();
	}

private:
	struct callback {
		StackTraceImpl& self;
		callback(StackTraceImpl& self): self(self) {}

		void operator()(size_t idx, void* addr) {
			self._stacktrace[idx] = addr;
		}
	};
};


#else 

template <>
class StackTraceImpl<system_tag::linux_tag>: public StackTraceLinuxImplHolder {
public:
	__attribute__ ((noinline)) // TODO use some macro
	size_t load_here(size_t depth=32) {
		load_thread_info();
		if (depth == 0) {
			return 0;
		}
		_stacktrace.resize(depth + 1);
		size_t trace_cnt = backtrace(&_stacktrace[0], _stacktrace.size());
		_stacktrace.resize(trace_cnt);
		skip_n_firsts(1);
		return size();
	}

	size_t load_from(void* addr, size_t depth=32) {
		load_here(depth + 8);

		for (size_t i = 0; i < _stacktrace.size(); ++i) {
			if (_stacktrace[i] == addr) {
				skip_n_firsts(i);
				_stacktrace[i] = (void*)( (uintptr_t)_stacktrace[i] + 1);
				break;
			}
		}

		_stacktrace.resize(std::min(_stacktrace.size(),
					skip_n_firsts() + depth));
		return size();
	}
};

#endif 
#endif 

class StackTrace:
	public StackTraceImpl<system_tag::current_tag> {};

/*************** TRACE RESOLVER ***************/

template <typename TAG>
class TraceResolverImpl;

#ifdef BACKWARD_SYSTEM_UNKNOWN

template <>
class TraceResolverImpl<system_tag::unknown_tag> {
public:
	template <class ST>
		void load_stacktrace(ST&) {}
	ResolvedTrace resolve(ResolvedTrace t) {
		return t;
	}
};

#endif

#ifdef BACKWARD_SYSTEM_LINUX

class TraceResolverLinuxImplBase {
protected:
	std::string demangle(const char* funcname) {
		return _demangler.demangle(funcname);
	}

private:
	details::demangler _demangler;
};

template <typename STACKTRACE_TAG>
class TraceResolverLinuxImpl;

#if BACKWARD_HAS_BACKTRACE_SYMBOL == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::backtrace_symbol>:
	public TraceResolverLinuxImplBase {
public:
	template <class ST>
		void load_stacktrace(ST& st) {
			using namespace details;
			if (st.size() == 0) {
				return;
			}
			_symbols.reset(
					backtrace_symbols(st.begin(), st.size())
					);
		}

	ResolvedTrace resolve(ResolvedTrace trace) {
		char* filename = _symbols[trace.idx];
		char* funcname = filename;
		while (*funcname && *funcname != '(') {
			funcname += 1;
		}
		trace.object_filename.assign(filename, funcname++);
		char* funcname_end = funcname;
		while (*funcname_end && *funcname_end != ')' && *funcname_end != '+') {
			funcname_end += 1;
		}
		*funcname_end = '\0';
		trace.object_function = this->demangle(funcname);
		trace.source.function = trace.object_function; 
		return trace;
	}

private:
	details::handle<char**> _symbols;
};

#endif 

#if BACKWARD_HAS_BFD == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::libbfd>:
	public TraceResolverLinuxImplBase {
public:
	TraceResolverLinuxImpl(): _bfd_loaded(false) {}

	template <class ST>
		void load_stacktrace(ST&) {}

	ResolvedTrace resolve(ResolvedTrace trace) {
		Dl_info symbol_info;

		if (!dladdr(trace.addr, &symbol_info)) {
			return trace; // dat broken trace...
		}

		if (symbol_info.dli_sname) {
			trace.object_function = demangle(symbol_info.dli_sname);
		}

		if (!symbol_info.dli_fname) {
			return trace;
		}

		trace.object_filename = symbol_info.dli_fname;
		bfd_fileobject& fobj = load_object_with_bfd(symbol_info.dli_fname);
		if (!fobj.handle) {
			return trace; 
		}


		find_sym_result* details_selected; 

		find_sym_result details_call_site = find_symbol_details(fobj,
				trace.addr, symbol_info.dli_fbase);
		details_selected = &details_call_site;

#if BACKWARD_HAS_UNWIND == 0
		
		find_sym_result details_adjusted_call_site = find_symbol_details(fobj,
				(void*) (uintptr_t(trace.addr) - 1),
				symbol_info.dli_fbase);

		
		if (details_call_site.found && details_adjusted_call_site.found) {
			
			details_selected = &details_adjusted_call_site;
			trace.addr = (void*) (uintptr_t(trace.addr) - 1);
		}

		if (details_selected == &details_call_site && details_call_site.found) {
			
			details_call_site = find_symbol_details(fobj, trace.addr,
					symbol_info.dli_fbase);
		}
#endif 

		if (details_selected->found) {
			if (details_selected->filename) {
				trace.source.filename = details_selected->filename;
			}
			trace.source.line = details_selected->line;

			if (details_selected->funcname) {
			
				trace.source.function = demangle(details_selected->funcname);

				if (!symbol_info.dli_sname) {
					
					trace.object_function = trace.source.function;
				}
			}

		
			trace.inliners = backtrace_inliners(fobj, *details_selected);

#if 0
			if (trace.inliners.size() == 0) {
				

				if (symbol_info.dli_saddr) {
					find_sym_result details = find_symbol_details(fobj,
							symbol_info.dli_saddr,
							symbol_info.dli_fbase);

					if (details.found) {
						ResolvedTrace::SourceLoc diy_inliner;
						diy_inliner.line = details.line;
						if (details.filename) {
							diy_inliner.filename = details.filename;
						}
						if (details.funcname) {
							diy_inliner.function = demangle(details.funcname);
						} else {
							diy_inliner.function = trace.source.function;
						}
						if (diy_inliner != trace.source) {
							trace.inliners.push_back(diy_inliner);
						}
					}
				}
			}
#endif
		}

		return trace;
	}

private:
	bool                _bfd_loaded;

	typedef details::handle<bfd*,
			details::deleter<bfd_boolean, bfd*, &bfd_close>
				> bfd_handle_t;

	typedef details::handle<asymbol**> bfd_symtab_t;


	struct bfd_fileobject {
		bfd_handle_t handle;
		bfd_vma      base_addr;
		bfd_symtab_t symtab;
		bfd_symtab_t dynamic_symtab;
	};

	typedef details::hashtable<std::string, bfd_fileobject>::type
		fobj_bfd_map_t;
	fobj_bfd_map_t      _fobj_bfd_map;

	bfd_fileobject& load_object_with_bfd(const std::string& filename_object) {
		using namespace details;

		if (!_bfd_loaded) {
			using namespace details;
			bfd_init();
			_bfd_loaded = true;
		}

		fobj_bfd_map_t::iterator it =
			_fobj_bfd_map.find(filename_object);
		if (it != _fobj_bfd_map.end()) {
			return it->second;
		}

	
		bfd_fileobject& r = _fobj_bfd_map[filename_object];

	
		bfd_handle_t bfd_handle;

		int fd = open(filename_object.c_str(), O_RDONLY);
		bfd_handle.reset(
				bfd_fdopenr(filename_object.c_str(), "default", fd)
				);
		if (!bfd_handle) {
			close(fd);
			return r;
		}

		if (!bfd_check_format(bfd_handle.get(), bfd_object)) {
			return r; 
		}

		if ((bfd_get_file_flags(bfd_handle.get()) & HAS_SYMS) == 0) {
			return r; 
		}

		ssize_t symtab_storage_size =
			bfd_get_symtab_upper_bound(bfd_handle.get());

		ssize_t dyn_symtab_storage_size =
			bfd_get_dynamic_symtab_upper_bound(bfd_handle.get());

		if (symtab_storage_size <= 0 && dyn_symtab_storage_size <= 0) {
			return r; 
		}

		bfd_symtab_t symtab, dynamic_symtab;
		ssize_t symcount = 0, dyn_symcount = 0;

		if (symtab_storage_size > 0) {
			symtab.reset(
					(bfd_symbol**) malloc(symtab_storage_size)
					);
			symcount = bfd_canonicalize_symtab(
					bfd_handle.get(), symtab.get()
					);
		}

		if (dyn_symtab_storage_size > 0) {
			dynamic_symtab.reset(
					(bfd_symbol**) malloc(dyn_symtab_storage_size)
					);
			dyn_symcount = bfd_canonicalize_dynamic_symtab(
					bfd_handle.get(), dynamic_symtab.get()
					);
		}


		if (symcount <= 0 && dyn_symcount <= 0) {
			return r; 
		}

		r.handle = move(bfd_handle);
		r.symtab = move(symtab);
		r.dynamic_symtab = move(dynamic_symtab);
		return r;
	}

	struct find_sym_result {
		bool found;
		const char* filename;
		const char* funcname;
		unsigned int line;
	};

	struct find_sym_context {
		TraceResolverLinuxImpl* self;
		bfd_fileobject* fobj;
		void* addr;
		void* base_addr;
		find_sym_result result;
	};

	find_sym_result find_symbol_details(bfd_fileobject& fobj, void* addr,
			void* base_addr) {
		find_sym_context context;
		context.self = this;
		context.fobj = &fobj;
		context.addr = addr;
		context.base_addr = base_addr;
		context.result.found = false;
		bfd_map_over_sections(fobj.handle.get(), &find_in_section_trampoline,
				(void*)&context);
		return context.result;
	}

	static void find_in_section_trampoline(bfd*, asection* section,
			void* data) {
		find_sym_context* context = static_cast<find_sym_context*>(data);
		context->self->find_in_section(
				reinterpret_cast<bfd_vma>(context->addr),
				reinterpret_cast<bfd_vma>(context->base_addr),
				*context->fobj,
				section, context->result
				);
	}

	void find_in_section(bfd_vma addr, bfd_vma base_addr,
			bfd_fileobject& fobj, asection* section, find_sym_result& result)
	{
		if (result.found) return;

		if ((bfd_get_section_flags(fobj.handle.get(), section)
					& SEC_ALLOC) == 0)
			return; // a debug section is never loaded automatically.

		bfd_vma sec_addr = bfd_get_section_vma(fobj.handle.get(), section);
		bfd_size_type size = bfd_get_section_size(section);

		// are we in the boundaries of the section?
		if (addr < sec_addr || addr >= sec_addr + size) {
			addr -= base_addr; // oups, a relocated object, lets try again...
			if (addr < sec_addr || addr >= sec_addr + size) {
				return;
			}
		}

		if (!result.found && fobj.symtab) {
			result.found = bfd_find_nearest_line(fobj.handle.get(), section,
					fobj.symtab.get(), addr - sec_addr, &result.filename,
					&result.funcname, &result.line);
		}

		if (!result.found && fobj.dynamic_symtab) {
			result.found = bfd_find_nearest_line(fobj.handle.get(), section,
					fobj.dynamic_symtab.get(), addr - sec_addr,
					&result.filename, &result.funcname, &result.line);
		}

	}

	ResolvedTrace::source_locs_t backtrace_inliners(bfd_fileobject& fobj,
			find_sym_result previous_result) {
		
		ResolvedTrace::source_locs_t results;
		while (previous_result.found) {
			find_sym_result result;
			result.found = bfd_find_inliner_info(fobj.handle.get(),
					&result.filename, &result.funcname, &result.line);

			if (result.found)  {
				ResolvedTrace::SourceLoc src_loc;
				src_loc.line = result.line;
				if (result.filename) {
					src_loc.filename = result.filename;
				}
				if (result.funcname) {
					src_loc.function = demangle(result.funcname);
				}
				results.push_back(src_loc);
			}
			previous_result = result;
		}
		return results;
	}

	bool cstrings_eq(const char* a, const char* b) {
		if (!a || !b) {
			return false;
		}
		return strcmp(a, b) == 0;
	}

};
#endif

#if BACKWARD_HAS_DW == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::libdw>:
	public TraceResolverLinuxImplBase {
public:
	TraceResolverLinuxImpl(): _dwfl_handle_initialized(false) {}

	template <class ST>
		void load_stacktrace(ST&) {}

	ResolvedTrace resolve(ResolvedTrace trace) {
		using namespace details;

		Dwarf_Addr trace_addr = (Dwarf_Addr) trace.addr;

		if (!_dwfl_handle_initialized) {
			
			_dwfl_cb.reset(new Dwfl_Callbacks);
			_dwfl_cb->find_elf = &dwfl_linux_proc_find_elf;
			_dwfl_cb->find_debuginfo = &dwfl_standard_find_debuginfo;
			_dwfl_cb->debuginfo_path = 0;

			_dwfl_handle.reset(dwfl_begin(_dwfl_cb.get()));
			_dwfl_handle_initialized = true;

			if (!_dwfl_handle) {
				return trace;
			}

			
			dwfl_report_begin(_dwfl_handle.get());
			int r = dwfl_linux_proc_report (_dwfl_handle.get(), getpid());
			dwfl_report_end(_dwfl_handle.get(), NULL, NULL);
			if (r < 0) {
				return trace;
			}
		}

		if (!_dwfl_handle) {
			return trace;
		}

		Dwfl_Module* mod = dwfl_addrmodule(_dwfl_handle.get(), trace_addr);
		if (mod) {
			
			const char* module_name = dwfl_module_info (mod,
					0, 0, 0, 0, 0, 0, 0);
			if (module_name) {
				trace.object_filename = module_name;
			}
			
			const char* sym_name = dwfl_module_addrname(mod, trace_addr);
			if (sym_name) {
				trace.object_function = demangle(sym_name);
			}
		}

		Dwarf_Addr mod_bias = 0;
		Dwarf_Die* cudie = dwfl_module_addrdie(mod, trace_addr, &mod_bias);

#if 1
		if (!cudie) {
			
			while ((cudie = dwfl_module_nextcu(mod, cudie, &mod_bias))) {
				Dwarf_Die die_mem;
				Dwarf_Die* fundie = find_fundie_by_pc(cudie,
						trace_addr - mod_bias, &die_mem);
				if (fundie) {
					break;
				}
			}
		}
#endif


#ifdef BACKWARD_I_DO_NOT_RECOMMEND_TO_ENABLE_THIS_HORRIBLE_PIECE_OF_CODE
		if (!cudie) {
			

			Dwarf_Addr cfi_bias;
			Dwarf_CFI* cfi_cache = dwfl_module_eh_cfi(mod, &cfi_bias);

			Dwarf_Addr bias;
			while ((cudie = dwfl_module_nextcu(mod, cudie, &bias))) {
				if (dwarf_getsrc_die(cudie, trace_addr - bias)) {

					handle<Dwarf_Frame*> frame;
					dwarf_cfi_addrframe(cfi_cache, trace_addr - cfi_bias, &frame);
					if (frame) {
						break;
					}
				}
			}
		}
#endif

		if (!cudie) {
			return trace; 
		}

		Dwarf_Line* srcloc = dwarf_getsrc_die(cudie, trace_addr - mod_bias);

		if (srcloc) {
			const char* srcfile = dwarf_linesrc(srcloc, 0, 0);
			if (srcfile) {
				trace.source.filename = srcfile;
			}
			int line = 0, col = 0;
			dwarf_lineno(srcloc, &line);
			dwarf_linecol(srcloc, &col);
			trace.source.line = line;
			trace.source.col = col;
		}

		deep_first_search_by_pc(cudie, trace_addr - mod_bias,
				inliners_search_cb(trace));
		if (trace.source.function.size() == 0) {
			
			trace.source.function = trace.object_function;
		}

		return trace;
	}

private:
	typedef details::handle<Dwfl*, details::deleter<void, Dwfl*, &dwfl_end> >
		dwfl_handle_t;
	details::handle<Dwfl_Callbacks*, details::default_delete<Dwfl_Callbacks*> >
		           _dwfl_cb;
	dwfl_handle_t  _dwfl_handle;
	bool           _dwfl_handle_initialized;

	
	struct inliners_search_cb {
		void operator()(Dwarf_Die* die) {
			switch (dwarf_tag(die)) {
				const char* name;
				case DW_TAG_subprogram:
					if ((name = dwarf_diename(die))) {
						trace.source.function = name;
					}
					break;

				case DW_TAG_inlined_subroutine:
					ResolvedTrace::SourceLoc sloc;
					Dwarf_Attribute attr_mem;

					if ((name = dwarf_diename(die))) {
						sloc.function = name;
					}
					if ((name = die_call_file(die))) {
						sloc.filename = name;
					}

					Dwarf_Word line = 0, col = 0;
					dwarf_formudata(dwarf_attr(die, DW_AT_call_line,
								&attr_mem), &line);
					dwarf_formudata(dwarf_attr(die, DW_AT_call_column,
								&attr_mem), &col);
					sloc.line = line;
					sloc.col = col;

					trace.inliners.push_back(sloc);
					break;
			};
		}
		ResolvedTrace& trace;
		inliners_search_cb(ResolvedTrace& t): trace(t) {}
	};


	static bool die_has_pc(Dwarf_Die* die, Dwarf_Addr pc) {
		Dwarf_Addr low, high;

		
		if (dwarf_hasattr(die, DW_AT_low_pc) and
							dwarf_hasattr(die, DW_AT_high_pc)) {
			if (dwarf_lowpc(die, &low) != 0) {
				return false;
			}
			if (dwarf_highpc(die, &high) != 0) {
				Dwarf_Attribute attr_mem;
				Dwarf_Attribute* attr = dwarf_attr(die, DW_AT_high_pc, &attr_mem);
				Dwarf_Word value;
				if (dwarf_formudata(attr, &value) != 0) {
					return false;
				}
				high = low + value;
			}
			return pc >= low && pc < high;
		}

		Dwarf_Addr base;
		ptrdiff_t offset = 0;
		while ((offset = dwarf_ranges(die, offset, &base, &low, &high)) > 0) {
			if (pc >= low && pc < high) {
				return true;
			}
		}
		return false;
	}

	static Dwarf_Die* find_fundie_by_pc(Dwarf_Die* parent_die, Dwarf_Addr pc,
			Dwarf_Die* result) {
		if (dwarf_child(parent_die, result) != 0) {
			return 0;
		}

		Dwarf_Die* die = result;
		do {
			switch (dwarf_tag(die)) {
				case DW_TAG_subprogram:
				case DW_TAG_inlined_subroutine:
					if (die_has_pc(die, pc)) {
						return result;
					}
				default:
					bool declaration = false;
					Dwarf_Attribute attr_mem;
					dwarf_formflag(dwarf_attr(die, DW_AT_declaration,
								&attr_mem), &declaration);
					if (!declaration) {
						
						Dwarf_Die die_mem;
						Dwarf_Die* indie = find_fundie_by_pc(die, pc, &die_mem);
						if (indie) {
							*result = die_mem;
							return result;
						}
					}
			};
		} while (dwarf_siblingof(die, result) == 0);
		return 0;
	}

	template <typename CB>
		static bool deep_first_search_by_pc(Dwarf_Die* parent_die,
				Dwarf_Addr pc, CB cb) {
		Dwarf_Die die_mem;
		if (dwarf_child(parent_die, &die_mem) != 0) {
			return false;
		}

		bool branch_has_pc = false;
		Dwarf_Die* die = &die_mem;
		do {
			bool declaration = false;
			Dwarf_Attribute attr_mem;
			dwarf_formflag(dwarf_attr(die, DW_AT_declaration, &attr_mem), &declaration);
			if (!declaration) {
				
				branch_has_pc = deep_first_search_by_pc(die, pc, cb);
			}
			if (!branch_has_pc) {
				branch_has_pc = die_has_pc(die, pc);
			}
			if (branch_has_pc) {
				cb(die);
			}
		} while (dwarf_siblingof(die, &die_mem) == 0);
		return branch_has_pc;
	}

	static const char* die_call_file(Dwarf_Die *die) {
		Dwarf_Attribute attr_mem;
		Dwarf_Sword file_idx = 0;

		dwarf_formsdata(dwarf_attr(die, DW_AT_call_file, &attr_mem),
				&file_idx);

		if (file_idx == 0) {
			return 0;
		}

		Dwarf_Die die_mem;
		Dwarf_Die* cudie = dwarf_diecu(die, &die_mem, 0, 0);
		if (!cudie) {
			return 0;
		}

		Dwarf_Files* files = 0;
		size_t nfiles;
		dwarf_getsrcfiles(cudie, &files, &nfiles);
		if (!files) {
			return 0;
		}

		return dwarf_filesrc(files, file_idx, 0, 0);
	}

};
#endif 

template<>
class TraceResolverImpl<system_tag::linux_tag>:
	public TraceResolverLinuxImpl<trace_resolver_tag::current> {};

#endif 

class TraceResolver:
	public TraceResolverImpl<system_tag::current_tag> {};

/*************** CODE SNIPPET ***************/

class SourceFile {
public:
	typedef std::vector<std::pair<unsigned, std::string> > lines_t;

	SourceFile() {}
	SourceFile(const std::string& path): _file(new std::ifstream(path.c_str())) {}
	bool is_open() const { return _file->is_open(); }

	lines_t& get_lines(unsigned line_start, unsigned line_count, lines_t& lines) {
		using namespace std;
		
		_file->clear();
		_file->seekg(0);
		string line;
		unsigned line_idx;

		for (line_idx = 1; line_idx < line_start; ++line_idx) {
			std::getline(*_file, line);
			if (!*_file) {
				return lines;
			}
		}

		
		struct isspace {
			bool operator()(char c) {
				return std::isspace(c);
			}
		};

		bool started = false;
		for (; line_idx < line_start + line_count; ++line_idx) {
			getline(*_file, line);
			if (!*_file) {
				return lines;
			}
			if (!started) {
				if (std::find_if(line.begin(), line.end(),
							not_isspace()) == line.end())
					continue;
				started = true;
			}
			lines.push_back(make_pair(line_idx, line));
		}

		lines.erase(
				std::find_if(lines.rbegin(), lines.rend(),
					not_isempty()).base(), lines.end()
				);
		return lines;
	}

	lines_t get_lines(unsigned line_start, unsigned line_count) {
		lines_t lines;
		return get_lines(line_start, line_count, lines);
	}

	struct not_isspace {
		bool operator()(char c) {
			return !std::isspace(c);
		}
	};

	struct not_isempty {
		bool operator()(const lines_t::value_type& p) {
			return !(std::find_if(p.second.begin(), p.second.end(),
						not_isspace()) == p.second.end());
		}
	};

	void swap(SourceFile& b) {
		_file.swap(b._file);
	}

#ifdef BACKWARD_ATLEAST_CXX11
	SourceFile(SourceFile&& from): _file(0) {
		swap(from);
	}
	SourceFile& operator=(SourceFile&& from) {
		swap(from); return *this;
	}
#else
	explicit SourceFile(const SourceFile& from) {
	
		swap(const_cast<SourceFile&>(from));
	}
	SourceFile& operator=(const SourceFile& from) {
		
		swap(const_cast<SourceFile&>(from)); return *this;
	}
#endif

private:
	details::handle<std::ifstream*,
		details::default_delete<std::ifstream*>
			> _file;

#ifdef BACKWARD_ATLEAST_CXX11
	SourceFile(const SourceFile&) = delete;
	SourceFile& operator=(const SourceFile&) = delete;
#endif
};

class SnippetFactory {
public:
	typedef SourceFile::lines_t lines_t;

	lines_t get_snippet(const std::string& filename,
			unsigned line_start, unsigned context_size) {

		SourceFile& src_file = get_src_file(filename);
		unsigned start = line_start - context_size / 2;
		return src_file.get_lines(start, context_size);
	}

	lines_t get_combined_snippet(
			const std::string& filename_a, unsigned line_a,
			const std::string& filename_b, unsigned line_b,
			unsigned context_size) {
		SourceFile& src_file_a = get_src_file(filename_a);
		SourceFile& src_file_b = get_src_file(filename_b);

		lines_t lines = src_file_a.get_lines(line_a - context_size / 4,
				context_size / 2);
		src_file_b.get_lines(line_b - context_size / 4, context_size / 2,
				lines);
		return lines;
	}

	lines_t get_coalesced_snippet(const std::string& filename,
			unsigned line_a, unsigned line_b, unsigned context_size) {
		SourceFile& src_file = get_src_file(filename);

		using std::min; using std::max;
		unsigned a = min(line_a, line_b);
		unsigned b = max(line_a, line_b);

		if ((b - a) < (context_size / 3)) {
			return src_file.get_lines((a + b - context_size + 1) / 2,
					context_size);
		}

		lines_t lines = src_file.get_lines(a - context_size / 4,
				context_size / 2);
		src_file.get_lines(b - context_size / 4, context_size / 2, lines);
		return lines;
	}


private:
	typedef details::hashtable<std::string, SourceFile>::type src_files_t;
	src_files_t _src_files;

	SourceFile& get_src_file(const std::string& filename) {
		src_files_t::iterator it = _src_files.find(filename);
		if (it != _src_files.end()) {
			return it->second;
		}
		SourceFile& new_src_file = _src_files[filename];
		new_src_file = SourceFile(filename);
		return new_src_file;
	}
};

/*************** PRINTER ***************/

#ifdef BACKWARD_SYSTEM_LINUX

namespace Color {
	enum type {
		yellow = 33,
		purple = 35,
		reset  = 39
	};
} // namespace Color

class Colorize {
public:
	Colorize(std::FILE* os):
		_os(os), _reset(false), _istty(false) {}

	void init() {
		_istty = isatty(fileno(_os));
	}

	void set_color(Color::type ccode) {
		if (!_istty) return;

		fprintf(_os, "\033[%im", static_cast<int>(ccode));
		_reset = (ccode != Color::reset);
	}

	~Colorize() {
		if (_reset) {
			set_color(Color::reset);
		}
	}

private:
	std::FILE* _os;
	bool       _reset;
	bool       _istty;
};

#else 


namespace Color {
	enum type {
		yellow = 0,
		purple = 0,
		reset  = 0
	};
} 

class Colorize {
public:
	Colorize(std::FILE*) {}
	void init() {}
	void set_color(Color::type) {}
};

#endif 

class Printer {
public:
	bool snippet;
	bool color;
	bool address;
	bool object;

	Printer():
		snippet(true),
		color(true),
		address(false),
		object(false)
		{}

	template <typename ST>
		FILE* print(ST& st, FILE* os = stderr) {
			Colorize colorize(os);
			if (color) {
				colorize.init();
			}
			print_header(os, st.thread_id());
			_resolver.load_stacktrace(st);
			for (size_t trace_idx = st.size(); trace_idx > 0; --trace_idx) {
				print_trace(os, _resolver.resolve(st[trace_idx-1]), colorize);
			}
			return os;
		}

	template <typename IT>
		FILE* print(IT begin, IT end, FILE* os = stderr, size_t thread_id = 0) {
			Colorize colorize(os);
			if (color) {
				colorize.init();
			}
			print_header(os, thread_id);
			for (; begin != end; ++begin) {
				print_trace(os, *begin, colorize);
			}
			return os;
		}
private:
	TraceResolver  _resolver;
	SnippetFactory _snippets;

	void print_header(FILE* os, unsigned thread_id) {
		fprintf(os, "Stack trace (most recent call last)");
		if (thread_id) {
			fprintf(os, " in thread %u:\n", thread_id);
		} else {
			fprintf(os, ":\n");
		}
	}

	void print_trace(FILE* os, const ResolvedTrace& trace,
			Colorize& colorize) {
		fprintf(os, "#%-2u", trace.idx);
		bool already_indented = true;

		if (!trace.source.filename.size() || object) {
			fprintf(os, "   Object \"%s\", at %p, in %s\n",
					trace.object_filename.c_str(), trace.addr,
					trace.object_function.c_str());
			already_indented = false;
		}

		for (size_t inliner_idx = trace.inliners.size();
				inliner_idx > 0; --inliner_idx) {
			if (!already_indented) {
				fprintf(os, "   ");
			}
			const ResolvedTrace::SourceLoc& inliner_loc
				= trace.inliners[inliner_idx-1];
			print_source_loc(os, " | ", inliner_loc);
			if (snippet) {
				print_snippet(os, "    | ", inliner_loc,
						colorize, Color::purple, 5);
			}
			already_indented = false;
		}

		if (trace.source.filename.size()) {
			if (!already_indented) {
				fprintf(os, "   ");
			}
			print_source_loc(os, "   ", trace.source, trace.addr);
			if (snippet) {
				print_snippet(os, "      ", trace.source,
						colorize, Color::yellow, 7);
			}
		}
	}

	void print_snippet(FILE* os, const char* indent,
			const ResolvedTrace::SourceLoc& source_loc,
			Colorize& colorize, Color::type color_code,
			int context_size)
	{
		using namespace std;
		typedef SnippetFactory::lines_t lines_t;

		lines_t lines = _snippets.get_snippet(source_loc.filename,
				source_loc.line, context_size);

		for (lines_t::const_iterator it = lines.begin();
				it != lines.end(); ++it) {
			if (it-> first == source_loc.line) {
				colorize.set_color(color_code);
				fprintf(os, "%s>", indent);
			} else {
				fprintf(os, "%s ", indent);
			}
			fprintf(os, "%4u: %s\n", it->first, it->second.c_str());
			if (it-> first == source_loc.line) {
				colorize.set_color(Color::reset);
			}
		}
	}

	void print_source_loc(FILE* os, const char* indent,
			const ResolvedTrace::SourceLoc& source_loc,
			void* addr=0) {
		fprintf(os, "%sSource \"%s\", line %i, in %s",
				indent, source_loc.filename.c_str(), (int)source_loc.line,
				source_loc.function.c_str());

		if (address && addr != 0) {
			fprintf(os, " [%p]\n", addr);
		} else {
			fprintf(os, "\n");
		}
	}
};

/*************** SIGNALS HANDLING ***************/

#ifdef BACKWARD_SYSTEM_LINUX


class SignalHandling {
public:
   static std::vector<int> make_default_signals() {
       const int posix_signals[] = {
		
		SIGILL,
		SIGABRT,
		SIGFPE,
		SIGSEGV,
		SIGBUS,
		
		SIGHUP,
		SIGINT,
		SIGPIPE,
		SIGALRM,
		SIGTERM,
		SIGUSR1,
		SIGUSR2,
		SIGPOLL,
		SIGPROF,
		SIGVTALRM,
		SIGIO,
		SIGPWR,
		
		SIGQUIT,
		SIGSYS,
		SIGTRAP,
		SIGXCPU,
		SIGXFSZ
	};
        return std::vector<int>(posix_signals, posix_signals + sizeof posix_signals / sizeof posix_signals[0] );
   }

  SignalHandling(const std::vector<int>& posix_signals = make_default_signals()):
	  _loaded(false) {
		bool success = true;

		const size_t stack_size = 1024 * 1024 * 8;
		_stack_content.reset((char*)malloc(stack_size));
		if (_stack_content) {
			stack_t ss;
			ss.ss_sp = _stack_content.get();
			ss.ss_size = stack_size;
			ss.ss_flags = 0;
			if (sigaltstack(&ss, 0) < 0) {
				success = false;
			}
		} else {
			success = false;
		}

		for (size_t i = 0; i < posix_signals.size(); ++i) {
			struct sigaction action;
			memset(&action, 0, sizeof action);
			action.sa_flags = (SA_SIGINFO | SA_ONSTACK | SA_NODEFER |
					SA_RESETHAND);
			sigfillset(&action.sa_mask);
			sigdelset(&action.sa_mask, posix_signals[i]);
			action.sa_sigaction = &sig_handler;

			int r = sigaction(posix_signals[i], &action, 0);
			if (r < 0) success = false;
		}

		_loaded = success;
	}

	bool loaded() const { return _loaded; }

private:
	details::handle<char*> _stack_content;
	bool                   _loaded;

	static void sig_handler(int, siginfo_t* info, void* _ctx) {
		ucontext_t *uctx = (ucontext_t*) _ctx;

		StackTrace st;
		void* error_addr = 0;
#ifdef REG_RIP 
		error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.gregs[REG_RIP]);
#elif defined(REG_EIP) 
		error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.gregs[REG_EIP]);
#elif defined(__arm__)
		error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.arm_pc);
#else
#	warning ":/ sorry, ain't know no nothing none not of your architecture!"
#endif
		if (error_addr) {
			st.load_from(error_addr, 32);
		} else {
			st.load_here(32);
		}

		Printer printer;
		printer.address = true;
		printer.print(st, stderr);

#if _XOPEN_SOURCE >= 700 || _POSIX_C_SOURCE >= 200809L
		psiginfo(info, 0);
#endif

		
		raise(info->si_signo);

		
		puts("watf? exit");
		_exit(EXIT_FAILURE);
	}
};

#endif 

#ifdef BACKWARD_SYSTEM_UNKNOWN

class SignalHandling {
public:
	SignalHandling(const std::vector<int>& = std::vector<int>()) {}
	bool init() { return false; }
	bool loaded() { return false; }
};

#endif 

} 

#endif /* H_GUARD */
