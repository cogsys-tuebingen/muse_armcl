#ifndef CONTACT_EVALUATION_DATA_HPP
#define CONTACT_EVALUATION_DATA_HPP
#include <map>
#include <vector>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <muse_armcl/evaluation/contact_sample.hpp>
#include <muse_armcl/evaluation/msgs_conversion.hpp>

namespace muse_armcl {
template <typename T>
class TimedData
{
public:
    TimedData() {}

    T& operator[](uint64_t& time)
    {
        std::size_t id = time_map_[time];
        return data_[id];
    }

    T& at(const uint64_t& time)
    {
        std::size_t id = time_map_.at(time);
        return data_.at(id);
    }

    const T& at(const uint64_t& time) const
    {
        std::size_t id = time_map_.at(time);
        return data_.at(id);
    }

    bool emplace_back(const uint64_t& time, const T& d)
    {
        auto it = time_map_.find(time);
        if(it == time_map_.end()){
            std::size_t id = data_.size();
            data_.emplace_back(d);
            time_map_[time] = id;
            return true;
        }
        return false;
    }

    bool push_back(const uint64_t& time, const T& d)
    {
        auto it = time_map_.find(time);
        if(it == time_map_.end()){
            std::size_t id = data_.size();
            data_.emplace_back(d);
            time_map_[time] = id;
            return true;
        }
        return false;
    }

    std::size_t size() const
    {
        return data_.size();
    }

    typename std::vector<T>::iterator begin()
    {
        return data_.begin();
    }

    typename std::vector<T>::iterator end()
    {
        return data_.end();
    }

    typename std::vector<T>::const_iterator begin() const
    {
        return data_.begin();
    }

    typename std::vector<T>::const_iterator end() const
    {
        return data_.end();
    }



protected:
    std::map<uint64_t,std::size_t> time_map_;
    std::vector<T> data_;
};

class ContactSequence : public TimedData<ContactSample>
{
public:
    ContactSequence() {}

    void setData(const jaco2_contact_msgs::Jaco2CollisionSequence& data)
    {
        time_map_.clear();
        data_.clear();

        for(const jaco2_contact_msgs::Jaco2CollisionSample& s : data.data){
            uint64_t nsecs = s.state.header.stamp.toNSec();
            std::size_t id = data_.size();
            time_map_[nsecs] = id;
            ContactSample tmp;
            convert(s, tmp);
            data_.emplace_back(tmp);
        }
    }
};

struct ContactEvaluationSample{
    std::vector<tf::StampedTransform> transforms;
    ContactSequence data;
};

class DataSet : public TimedData<ContactEvaluationSample>
{
public:
    DataSet(){}

    void setTf(uint64_t time, const tf::tfMessage& tf_msg)
    {
        std::vector<tf::StampedTransform> tf_vec;
        for(auto t : tf_msg.transforms){
            tf::StampedTransform tmp;
            tf::transformStampedMsgToTF(t, tmp);
            tf_vec.emplace_back(tmp);
        }

        auto it = time_map_.find(time);
        if(it == time_map_.end()){
            std::size_t id = data_.size();
            ContactEvaluationSample s;
            s.transforms = tf_vec;
            data_.emplace_back(s);
            time_map_[time] = id;
        } else {
            data_[it->second].transforms = tf_vec;
        }
    }

    void setContactData(uint64_t time, const jaco2_contact_msgs::Jaco2CollisionSequence& data)
    {
        ContactSequence seq;
        seq.setData(data);
        auto it = time_map_.find(time);
        if(it == time_map_.end()){
            std::size_t id = data_.size();
            ContactEvaluationSample s;
            s.data = seq;
            data_.emplace_back(s);
            time_map_[time] = id;
        } else {
            data_[it->second].data = seq;
        }

    }
};

}
#endif // CONTACT_EVALUATION_DATA_HPP
