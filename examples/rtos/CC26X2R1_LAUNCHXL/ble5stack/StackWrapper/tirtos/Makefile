IAR_PATH="C:\Program Files\IAR Systems\EWARM-9.60.2\common\bin\IarBuild.exe"
EWP_PATH="$(CURDIR)\iar\stack_wrapper.ewp"
VARFILE="$(CURDIR)\iar\stack_wrapper.custom_argvars"

26r1: clean_26r1 Stack_Wrapper_26r1

13r1: clean_13r1 Stack_Wrapper_13r1

13p2: clean_13p2 Stack_Wrapper_13p2

all: 26r1 13r1 13p2

Stack_Wrapper_26r1:
	$(IAR_PATH) $(EWP_PATH) -build Stack_Wrapper -varfile $(VARFILE)

Stack_Wrapper_13r1:
	$(IAR_PATH) $(EWP_PATH) -build Stack_Wrapper_13r1 -varfile $(VARFILE)

Stack_Wrapper_13p2:
	$(IAR_PATH) $(EWP_PATH) -build Stack_Wrapper_13p2 -varfile $(VARFILE)

clean_26r1:
	$(IAR_PATH) $(EWP_PATH) -clean Stack_Wrapper -varfile $(VARFILE)

clean_13r1:
	$(IAR_PATH) $(EWP_PATH) -clean Stack_Wrapper_13r1 -varfile $(VARFILE)

clean_13p2:
	$(IAR_PATH) $(EWP_PATH) -clean Stack_Wrapper_13p2 -varfile $(VARFILE)
