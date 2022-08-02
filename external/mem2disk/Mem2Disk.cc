#include <unordered_map>
#include <queue>
#include <filesystem>

#include "Mem2Disk.hpp"

using m2d::IDataObj;
using m2d::DataManager;

class DataManager::Impl {
public:
	using DataQueue = std::queue<IDataObj::Ptr>;

	std::queue<IDataObj::Ptr> data_pool_;
	std::filesystem::path root_;
};

std::unique_ptr<DataManager::Impl> DataManager::impl_(new DataManager::Impl);

void DataManager::DumpAll() {
	Impl::DataQueue& q = impl_->data_pool_;
	while (!q.empty()) {
		IDataObj::Ptr p = q.front();
		p->DumpToDisk();

		q.pop();
	}
}

void DataManager::Add(IDataObj::Ptr obj) {
	impl_->data_pool_.push(obj);
}

void DataManager::SetRoot(const std::string& root) {
	using std::filesystem::exists;

	impl_->root_ = std::filesystem::path(root);
	if (!exists(impl_->root_)) {
		std::filesystem::create_directories(impl_->root_);
	}
}

const std::string& DataManager::GetRoot() {
	return impl_->root_.string();
}