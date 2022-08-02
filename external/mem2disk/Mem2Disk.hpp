#pragma once

#include <memory>
#include <string>

namespace m2d {

	class IDataObj {
	public:
		using Ptr = std::shared_ptr<IDataObj>;

		virtual ~IDataObj() = default;
		virtual void DumpToDisk() = 0;
	};

	class DataManager {
	public:
		DataManager() = delete;

		static void SetRoot(const std::string &root);
		static const std::string& GetRoot();

		static void Add(IDataObj::Ptr obj);

		static void DumpAll();
	private:
		class Impl;
		static std::unique_ptr<Impl> impl_;
	};
}