#ifndef CONTACT_EVALUATION_DATA_HPP
#define CONTACT_EVALUATION_DATA_HPP
#include <map>
#include <vector>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <muse_armcl/evaluation/contact_sample.hpp>
#include <muse_armcl/evaluation/msgs_conversion.hpp>

#include <cslibs_time/time.hpp>

namespace muse_armcl {
template <typename T>
class TimedData
{
public:
    TimedData() :
        min_time_(std::numeric_limits<uint64_t>::max()),
        max_time_(std::numeric_limits<uint64_t>::min())
    {}

    T& at(const uint64_t& time)
    {
        std::size_t id = time_to_index_.at(time);
        return data_.at(id);
    }

    const T& at(const uint64_t& time) const
    {
        std::size_t id = time_to_index_.at(time);
        return data_.at(id);
    }

    std::size_t getID(const uint64_t time) const
    {
        return time_to_index_.at(time);
    }

    bool emplace_back(const uint64_t time, const T& d)
    {
        auto it = time_to_index_.find(time);
        if(it == time_to_index_.end()){
            std::size_t id = data_.size();
            data_.emplace_back(d);
            time_to_index_[time] = id;
            index_to_time_[id] = time;

            if(time < min_time_)
                min_time_ = time;
            if(time > max_time_)
                max_time_ = time;

            return true;
        }
        return false;
    }

    bool push_back(const uint64_t& time, const T& d)
    {
        auto it = time_to_index_.find(time);
        if(it == time_to_index_.end()){
            std::size_t id = data_.size();
            data_.emplace_back(d);
            time_to_index_[time] = id;
            index_to_time_[id] = time;

            if(time < min_time_)
                min_time_ = time;
            if(time > max_time_)
                max_time_ = time;

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

    uint64_t getMinTime() const
    {
        return min_time_;
    }

    uint64_t getMaxTime() const
    {
        return max_time_;
    }

    uint64_t getTimeFromIndex(const std::size_t index) const
    {
        return index_to_time_.at(index);
    }

    bool contains(const uint64_t time) const
    {
        return time_to_index_.find(time) != time_to_index_.end();
    }

protected:
    uint64_t                        min_time_;
    uint64_t                        max_time_;
    std::map<uint64_t,std::size_t>  time_to_index_;
    std::map<std::size_t, uint64_t> index_to_time_;
    std::vector<T>                  data_;
};

class ContactSequence : public TimedData<ContactSample>
{
public:
    ContactSequence() {}

    void setData(const uint64_t bag_time, const jaco2_contact_msgs::Jaco2CollisionSequence& data)
    {
        time_to_index_.clear();
        data_.clear();
        min_time_ = std::numeric_limits<uint64_t>::max();
        max_time_ = std::numeric_limits<uint64_t>::min();

        uint64_t start_time = static_cast<uint64_t>(cslibs_time::Time::now().nanoseconds());
        for(const jaco2_contact_msgs::Jaco2CollisionSample& s : data.data){
            uint64_t time = s.state.header.stamp.toNSec();

            if(time == 0) {
                time = static_cast<uint64_t>(cslibs_time::Time::now().nanoseconds());

                if(bag_time != 0) {
                    time = bag_time + (time - start_time);
                }
            }

            std::size_t id = data_.size();
            time_to_index_[time] = id;
            index_to_time_[id] = time;

            if(time < min_time_)
                min_time_ = time;
            if(time > max_time_)
                max_time_ = time;

            ContactSample tmp;
            convert(s, tmp);
            data_.emplace_back(tmp);
        }
    }
};

struct ContactEvaluationSample{

    ContactEvaluationSample() :
        has_transform(false),
        has_data(false)
    {}

    bool has_transform;
    bool has_data;
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

        auto it = time_to_index_.find(time);
        if(it == time_to_index_.end()){
            std::size_t id = data_.size();
            ContactEvaluationSample s;
            s.transforms = tf_vec;
            data_.emplace_back(s);
            time_to_index_[time] = id;

            if(time < min_time_)
                min_time_ = time;
            if(time > max_time_)
                max_time_ = time;

        } else {
            if(data_[it->second].has_transform){
                ++time;
                setTf(time, tf_msg);
            } else {
                data_[it->second].transforms = tf_vec;
                data_[it->second].has_transform = true;
            }
        }
    }

    void setContactData(uint64_t time, const jaco2_contact_msgs::Jaco2CollisionSequence& data)
    {
        ContactSequence seq;
        seq.setData(time, data);

        auto it = time_to_index_.find(time);
        if(it == time_to_index_.end()){
            std::size_t id = data_.size();
            ContactEvaluationSample s;
            s.data = seq;
            data_.emplace_back(s);
            time_to_index_[time] = id;

            if(time < min_time_)
                min_time_ = time;
            if(time > max_time_)
                max_time_ = time;

        } else {
            if(data_[it->second].has_data){
                ++time;
                setContactData(time, data);
            } else {
                data_[it->second].data = seq;
                data_[it->second].has_data = true;
            }
        }

    }
};

}
#endif // CONTACT_EVALUATION_DATA_HPP
