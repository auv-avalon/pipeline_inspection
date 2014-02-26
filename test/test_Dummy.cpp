#include <boost/test/unit_test.hpp>
#include <pipeline_inspection/Dummy.hpp>

using namespace pipeline_inspection;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    pipeline_inspection::DummyClass dummy;
    dummy.welcome();
}
