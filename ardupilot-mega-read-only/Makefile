#
# ArduPilot Mega sketchbook global makefile.
#
# This simple Makefile will build everything in the sketchbook with a Makefile at the top level
#

MAKE_DIRS	:=	$(dir $(wildcard */Makefile))

META_TARGETS	:=	all clean

$(META_TARGETS):
	@for dir in $(MAKE_DIRS) ; do  \
		echo "\n%%%% Making $@ in $$dir %%%%\n" ; \
		(cd $$dir && make -f Makefile $@) ; \
	done

DIR_TARGETS	:=	$(foreach dir,$(MAKE_DIRS),$(lastword $(subst /, ,$(dir))))

$(DIR_TARGETS):
	@(cd $@ && make -f Makefile)

DIR_META_TARGETS:=	$(foreach meta,$(META_TARGETS),$(addsuffix -$(meta),$(DIR_TARGETS)))

$(DIR_META_TARGETS):	dir  = $(word 1,$(subst -, ,$@))
$(DIR_META_TARGETS):	meta = $(word 2,$(subst -, ,$@))
$(DIR_META_TARGETS):
	@echo "\n%%%% Making $(meta) in $(dir) %%%%\n"
	@(cd $(dir) && make -f Makefile $(meta))

.PHONY:	$(META_TARGETS) $(DIR_TARGETS) $(DIR_META_TARGETS)
