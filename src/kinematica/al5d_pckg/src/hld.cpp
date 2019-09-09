#include "al5d_pckg/state.h"
#include "al5d_pckg/ConstValues.h"

hld::Ready::Ready(hld *m)
{
    this->onDo(m);
}

hld::Ready::~Ready()
{
}

void hld::Ready::onDo(hld *m)
{
    m->setCurrentStatename("READY");
    ROS_INFO("STATE: READY");
    m->startListening();
    if (m->getReceived())
    {
        ROS_DEBUG("EVENT: RECEIVED");
        m->setReceived(false);
        m->setCurrent(new Active(m));

        delete this;
    }
}

hld::Initialized::Initialized(hld *m)
{
    this->onDo(m);
}

hld::Initialized::~Initialized(){};

void hld::Initialized::onDo(hld *m)
{
    m->setCurrentStatename("INITIALIZED");
    ROS_INFO("STATE: INITIALIZED");
    //  this->onExit(m);
    if (!m->detectlowlvl())
    {
        ROS_DEBUG("EVENT: No robot arm  availabe");
        delete this;
        ros::shutdown();
    }
    else
    {
        ROS_DEBUG("EVENT: robot arm availabe");

        m->setCurrent(new Ready(m));
        delete this;
    }
}

hld::Active::Active(hld *m)
{
    this->onDo(m);
}

hld::Active::~Active()
{
}

void hld::Active::onDo(hld *m)
{
    m->setCurrentStatename("ACTIVE");
    ROS_INFO("STATE: ACTIVE");

    // ros::Rate r(1);
    bool success = false;
    double begin = ros::Time::now().toSec();
    double previoustime = ros::Time::now().toSec();

    if (m->goalptr->pose == 0)
    {
        m->setCurrentStatename("EMERGENCY");
        ROS_DEBUG("EVENT: ERMERGENCY");
        m->lld.sendAction(m->goalptr->pose, m->goalptr->servo, m->goalptr->angle, m->goalptr->duration, m->goalptr->size);
        m->setCurrent(new Emergency(m));
        delete this;
    }

    if (m->goalptr->pose == 1)
    {
        // publish info to the console for the user
        ROS_DEBUG("EVENT: Executing pose");

        for (int i = 0; i < m->goalptr->size; ++i)
        {

            m->lld.sendAction(m->goalptr->pose, m->goalptr->servo, m->goalptr->angle, m->goalptr->duration, m->goalptr->size);
            ROS_DEBUG("servo: %i", m->goalptr->servo[i]);
            ROS_DEBUG("angle: %i", m->goalptr->angle[i]);
            ROS_DEBUG("duration: %i", m->goalptr->duration);
        }

        while (ros::ok())
        {
            double now = ros::Time::now().toSec();

            if (m->isPreempt() || !ros::ok() && m->goalptr->pose == 1)
            {
                ROS_INFO("%s: Preempted", m->getActionName().c_str());
                // set the action state to preempted
                m->setPreempted();
                success = false;
                break;
            }
            if ((now - previoustime) == 1)
            {
                previoustime = now;
            }
            success = true;
            break;
        }

        if (success)
        {
            m->setResult();
            ROS_INFO("%s: Succeeded", m->getPosName().c_str());
            // set the action state to succeeded
            m->setSucceeded();
            if (m->getCurrentStatename() == "ACTIVE")
            {
                m->setCurrent(new Ready(m));
                delete this;
            }
        }
    }
}

hld::Emergency::Emergency(hld *m)
{
    this->onDo(m);
}

hld::Emergency::~Emergency()
{
}

void hld::Emergency::onDo(hld *m)
{
    m->setCurrentStatename("EMERGENCY");
    m->setResult();
    m->setAborted();
    ROS_DEBUG("STATE: EMERGENCY");
    ROS_ERROR("NOODSTOP");
    ros::shutdown();
}

hld::hld::hld(std::string name, std::string port) : as_(nh_, name, boost::bind(&hld::hld::executeCB, this, _1), false), action_name_(name)
{
    this->lld.setPort(port);
    mRosNode = std::make_unique<ros::NodeHandle>("al5d_pckg");
    this->mRosSubCommands =
        this->mRosNode->subscribe("/moveServo", 1,
                                  &hld::hld::CallBack, this);
    this->current = new Initialized(this);
}

hld::hld::~hld()
{
}

void hld::hld::setCurrent(State *s)
{
    this->current = s;
}

void hld::hld::setCurrentStatename(std::string s)
{
    this->currentStatename = s;
}

bool hld::hld::detectlowlvl()
{
    return (this->lld.detectDriver());
}

void hld::hld::executeCB(const al5d_pckg::communicatorGoalConstPtr &goal)
{
    this->goalptr = goal;
    if (this->getCurrentStatename() == "READY")
    {
        this->setReceived(true);
        this->setCurrent(new Ready(this));
    }
}

void hld::hld::startListening()
{
    as_.start();
}

void hld::hld::stopListening()
{
    as_.shutdown();
}

void hld::hld::setAborted()
{
    as_.setAborted(result_.result);
}

void hld::hld::setPreempted()
{
    as_.setPreempted();
}

void hld::hld::setSucceeded()
{
    as_.setSucceeded(result_.result);
}
void hld::hld::publishFeedback()
{
    as_.publishFeedback(feedback_.feedback);
}

bool hld::hld::isPreempt()
{
    return as_.isPreemptRequested();
}

void hld::hld::setReceived(const bool inputreceived)
{
    this->received = inputreceived;
}

const bool &hld::hld::getReceived()
{
    return this->received;
}

const std::string &hld::hld::getActionName()
{
    return this->action_name_;
}

const std::string &hld::hld::getPosName()
{
    return this->pose_name;
}

const std::string &hld::hld::getCurrentStatename()
{
    return this->currentStatename;
}

void hld::hld::setFeedback(int16_t angle, uint16_t time)
{
    feedback_.feedback.current_angle = angle;
    feedback_.feedback.current_time = time;
}
void hld::hld::setResult()
{
    this->result_.result.end_angle = this->feedback_.feedback.current_angle;
    this->result_.result.end_time = this->feedback_.feedback.current_time;
}

void hld::hld::CallBack(const al5d_pckg::moveArmConstPtr &aMsg)
{

    for (int i = 0; i < aMsg->servo.size(); ++i)
    {
        //std::cout << "succes" << std::endl;

          this->lld.sendAction(1, aMsg->servo, aMsg->angle, aMsg->duration, aMsg->servo.size());
        ROS_INFO("servo: %i", aMsg->servo[i]);
        ROS_INFO("angle: %i", aMsg->angle[i]);
        ROS_INFO("duration: %i", aMsg->duration);
    }
}