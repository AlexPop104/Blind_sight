//#line 2 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the pcl_tutorial package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __pcl_tutorial__PASSTHROUGH_FILTER_NODE_ZOOMCONFIG_H__
#define __pcl_tutorial__PASSTHROUGH_FILTER_NODE_ZOOMCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace pcl_tutorial
{
  class passthrough_filter_node_zoomConfigStatics;

  class passthrough_filter_node_zoomConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }

      virtual void clamp(passthrough_filter_node_zoomConfig &config, const passthrough_filter_node_zoomConfig &max, const passthrough_filter_node_zoomConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const passthrough_filter_node_zoomConfig &config1, const passthrough_filter_node_zoomConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, passthrough_filter_node_zoomConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const passthrough_filter_node_zoomConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, passthrough_filter_node_zoomConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const passthrough_filter_node_zoomConfig &config) const = 0;
      virtual void getValue(const passthrough_filter_node_zoomConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T passthrough_filter_node_zoomConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T passthrough_filter_node_zoomConfig::* field;

      virtual void clamp(passthrough_filter_node_zoomConfig &config, const passthrough_filter_node_zoomConfig &max, const passthrough_filter_node_zoomConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const passthrough_filter_node_zoomConfig &config1, const passthrough_filter_node_zoomConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, passthrough_filter_node_zoomConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const passthrough_filter_node_zoomConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, passthrough_filter_node_zoomConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const passthrough_filter_node_zoomConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const passthrough_filter_node_zoomConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, passthrough_filter_node_zoomConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, passthrough_filter_node_zoomConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<passthrough_filter_node_zoomConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(passthrough_filter_node_zoomConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("z_lower_limit"==(*_i)->name){z_lower_limit = boost::any_cast<double>(val);}
        if("z_upper_limit"==(*_i)->name){z_upper_limit = boost::any_cast<double>(val);}
        if("x_lower_limit"==(*_i)->name){x_lower_limit = boost::any_cast<double>(val);}
        if("x_upper_limit"==(*_i)->name){x_upper_limit = boost::any_cast<double>(val);}
        if("y_lower_limit"==(*_i)->name){y_lower_limit = boost::any_cast<double>(val);}
        if("y_upper_limit"==(*_i)->name){y_upper_limit = boost::any_cast<double>(val);}
      }
    }

    double z_lower_limit;
double z_upper_limit;
double x_lower_limit;
double x_upper_limit;
double y_lower_limit;
double y_upper_limit;

    bool state;
    std::string name;

    
}groups;



//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double z_lower_limit;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double z_upper_limit;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double x_lower_limit;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double x_upper_limit;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double y_lower_limit;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double y_upper_limit;
//#line 228 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("passthrough_filter_node_zoomConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const passthrough_filter_node_zoomConfig &__max__ = __getMax__();
      const passthrough_filter_node_zoomConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const passthrough_filter_node_zoomConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const passthrough_filter_node_zoomConfig &__getDefault__();
    static const passthrough_filter_node_zoomConfig &__getMax__();
    static const passthrough_filter_node_zoomConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const passthrough_filter_node_zoomConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void passthrough_filter_node_zoomConfig::ParamDescription<std::string>::clamp(passthrough_filter_node_zoomConfig &config, const passthrough_filter_node_zoomConfig &max, const passthrough_filter_node_zoomConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class passthrough_filter_node_zoomConfigStatics
  {
    friend class passthrough_filter_node_zoomConfig;

    passthrough_filter_node_zoomConfigStatics()
    {
passthrough_filter_node_zoomConfig::GroupDescription<passthrough_filter_node_zoomConfig::DEFAULT, passthrough_filter_node_zoomConfig> Default("Default", "", 0, 0, true, &passthrough_filter_node_zoomConfig::groups);
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.z_lower_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.z_lower_limit = 15.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.z_lower_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("z_lower_limit", "double", 1, "Z Lower limit", "", &passthrough_filter_node_zoomConfig::z_lower_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("z_lower_limit", "double", 1, "Z Lower limit", "", &passthrough_filter_node_zoomConfig::z_lower_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.z_upper_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.z_upper_limit = 50.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.z_upper_limit = 3.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("z_upper_limit", "double", 1, "Z Upper limit", "", &passthrough_filter_node_zoomConfig::z_upper_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("z_upper_limit", "double", 1, "Z Upper limit", "", &passthrough_filter_node_zoomConfig::z_upper_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.x_lower_limit = -1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.x_lower_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.x_lower_limit = -0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("x_lower_limit", "double", 1, "X Lower limit", "", &passthrough_filter_node_zoomConfig::x_lower_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("x_lower_limit", "double", 1, "X Lower limit", "", &passthrough_filter_node_zoomConfig::x_lower_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.x_upper_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.x_upper_limit = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.x_upper_limit = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("x_upper_limit", "double", 1, "X Upper limit", "", &passthrough_filter_node_zoomConfig::x_upper_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("x_upper_limit", "double", 1, "X Upper limit", "", &passthrough_filter_node_zoomConfig::x_upper_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.y_lower_limit = -1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.y_lower_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.y_lower_limit = -0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("y_lower_limit", "double", 1, "Y Lower limit", "", &passthrough_filter_node_zoomConfig::y_lower_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("y_lower_limit", "double", 1, "Y Lower limit", "", &passthrough_filter_node_zoomConfig::y_lower_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.y_upper_limit = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.y_upper_limit = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.y_upper_limit = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("y_upper_limit", "double", 1, "Y Upper limit", "", &passthrough_filter_node_zoomConfig::y_upper_limit)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr(new passthrough_filter_node_zoomConfig::ParamDescription<double>("y_upper_limit", "double", 1, "Y Upper limit", "", &passthrough_filter_node_zoomConfig::y_upper_limit)));
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(passthrough_filter_node_zoomConfig::AbstractGroupDescriptionConstPtr(new passthrough_filter_node_zoomConfig::GroupDescription<passthrough_filter_node_zoomConfig::DEFAULT, passthrough_filter_node_zoomConfig>(Default)));
//#line 366 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<passthrough_filter_node_zoomConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<passthrough_filter_node_zoomConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    passthrough_filter_node_zoomConfig __max__;
    passthrough_filter_node_zoomConfig __min__;
    passthrough_filter_node_zoomConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const passthrough_filter_node_zoomConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static passthrough_filter_node_zoomConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &passthrough_filter_node_zoomConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const passthrough_filter_node_zoomConfig &passthrough_filter_node_zoomConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const passthrough_filter_node_zoomConfig &passthrough_filter_node_zoomConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const passthrough_filter_node_zoomConfig &passthrough_filter_node_zoomConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<passthrough_filter_node_zoomConfig::AbstractParamDescriptionConstPtr> &passthrough_filter_node_zoomConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<passthrough_filter_node_zoomConfig::AbstractGroupDescriptionConstPtr> &passthrough_filter_node_zoomConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const passthrough_filter_node_zoomConfigStatics *passthrough_filter_node_zoomConfig::__get_statics__()
  {
    const static passthrough_filter_node_zoomConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = passthrough_filter_node_zoomConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __PASSTHROUGH_FILTER_NODE_ZOOMRECONFIGURATOR_H__
