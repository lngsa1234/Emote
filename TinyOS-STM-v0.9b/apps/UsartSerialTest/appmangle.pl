#!/usr/bin/perl -w

use strict;
use Getopt::Long;

my $file = '';
my $multi_match = 0;
my $enum_match = 0;


GetOptions(
	'file=s' => \$file,
);

if ( ! $file) {
	die "no valid file \n";
}

open(FILE,"<$file") or die "no such file $file\n";
while(<FILE>) {
  # If on the first line, print some header.  It's in the while loop so if
  # this script is invoked with -i, the correct things still happen.
  # NOTE: This was broken by SMAC!
  if( $. == 1 ) {
    print <<"EOF";

#define MANGLED_NESC_APP_C
EOF
  }

  s/__extension__//;
  
  #remove empty struct;
  
  $multi_match = 1 if /struct\s+__nesc_attr_atmostonce\s*\{/;
  if ( $multi_match && /\}/ ) {
    $multi_match = 0;
    next;
  }
  next if $multi_match;
  
  $multi_match = 1 if /struct\s+__nesc_attr_atleastonce\s*\{/;
  if ( $multi_match && /\}/ ) {
    $multi_match = 0;
    next;
  }
  next if $multi_match;
  
  $multi_match = 1 if /struct\s+__nesc_attr_exactlyonce\s*\{/;
  if ( $multi_match && /\}/ ) {
    $multi_match = 0;
    next;
  }
  next if $multi_match;
 


# disable file and line number preprocessor commands
  s{^(# \d+|#line)}{//$1};
  	
  s{^(.*UartC____nesc_sillytask_my_send.*)}{//$1};
  	print;
}
  close(FILE);