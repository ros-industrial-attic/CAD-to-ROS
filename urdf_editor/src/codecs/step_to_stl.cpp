#include "step_to_stl.h"

// OCE library headers
#include <Interface_Static.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <TCollection_AsciiString.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopoDS_Shape.hxx>

#include <ros/console.h>

bool urdf_editor::codecs::convertStepToStl(const std::string& step_file_path, 
                                           const std::string& stl_file_path)
{
  STEPControl_Reader step_reader;
  Interface_Static::SetCVal("xstep.cascade.unit", "M"); // units in Meters

  IFSelect_ReturnStatus ret = step_reader.ReadFile(step_file_path.c_str());

  if (ret != IFSelect_RetDone) 
  {
    ROS_WARN("Unable to load STEP file '%s'", step_file_path.c_str());
    return false;
  }

  Standard_Integer n = step_reader.TransferRoots();
  TopoDS_Shape shape = step_reader.OneShape();

  StlAPI_Writer writer;
  writer.ASCIIMode() = 0;
  writer.Write(shape, stl_file_path.c_str());

  // Our edition of OCE doesn't have a return code for the above write
  return true;
}
