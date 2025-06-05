#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/motoman.hh>

void vamp::binding::init_motoman(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Motoman>(pymodule);
}
