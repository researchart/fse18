#!/usr/bin/env perl
# Author: Jamie Davis <davisjam@vt.edu>
# Description: Help assess the results of a language guessing pass
#   Prints NUM_PER_LABEL randomly chosen specimens for each label
#   Use SEED for repeatability

use strict;
use warnings;

use JSON::PP;
use List::Util qw(shuffle);

# Check usage
if (scalar(@ARGV) < 3) {
  die "Usage: $0 labeled-results.json NUM_PER_LABEL MERGE_SIMILAR [SEED]\n";
}

# Arg parsing
my $file = $ARGV[0];
if (not -f $file) {
  die "Error, no such file <$file>\n";
}

my $NUM_PER_LABEL = $ARGV[1];
print "Using $NUM_PER_LABEL per label\n";

my $MERGE_SIMILAR = $ARGV[2];
if (not defined $MERGE_SIMILAR) {
  die "Error, must provide MERGE_SIMILAR: 0|1";
}
$MERGE_SIMILAR = int($MERGE_SIMILAR);
print "Using MERGE_SIMILAR $MERGE_SIMILAR\n";

my $SEED = $ARGV[2];
if (not defined $SEED) {
  $SEED = time;
}
print "Using SEED $SEED\n";
srand($SEED);

# Load results.
print "Loading results\n";
my $lines = &readFile("file"=>$file);
my @lines = split("\n", $lines);
chomp @lines;
my @res = map { decode_json($_) } @lines;

# Bin by language.
my %lang2entries;
for my $entry (@res) {
  if ($MERGE_SIMILAR) {
    if ($entry->{language} eq "file" or $entry->{language} eq "filetype") {
      $entry->{language} = "file/path";
    }
    if ($entry->{language} eq "js") {
      $entry->{language} = "source-code";
    }
  }

  my $lang = $entry->{language};

  if (not defined $lang2entries{$lang}) {
    $lang2entries{$lang} = [];
  }

  push @{$lang2entries{$lang}}, $entry;
}

my @langs = sort keys %lang2entries;

# How many of each language?
printf("\n  %16s    %s\n", "Language", "Number of patterns");
print "  ----------------------------------------\n";
my $totalPatterns = 0;
my $nClassified = 0;
for my $lang (@langs) {
	my $nPatterns = scalar(@{$lang2entries{$lang}});
	printf("  %16s    %d\n", $lang, $nPatterns);

	$totalPatterns += $nPatterns;
	if ($lang !~ m/UNKNOWN/) {
		$nClassified += $nPatterns;
	}
}
print "         ---------------------\n";
printf("  %16s    %d\n", "all", $totalPatterns);
print "         ---------------------\n";
my $percClassified = sprintf("%.2f", 100 * ($nClassified / $totalPatterns));
print "  $percClassified% classified ($nClassified/$totalPatterns)\n";

print "\nTaking sample...\n\n";

# Go in key-sorted order to ensure the RNG is accessed in the same way each time.
for my $lang (@langs) {
  my @subset = &randomSubset($lang2entries{$lang}, $NUM_PER_LABEL);
	my $sampleSize = scalar(@subset);
	if ($sampleSize < $NUM_PER_LABEL) {
  	print "All $sampleSize patterns classified as $lang:\n";
	}
	else {
  	print "$sampleSize patterns classified as $lang:\n";
	}
  for my $e (@subset) {
		print "  " . encode_json({"pattern"=>$e->{pattern}}) . "\n";
  }
	print "\n";
}

exit 0;

################

# input: (\@list, $n)
# output: random subset of @list of size $n (or entire list if $n is larger)
sub randomSubset {
  my ($list, $n) = @_;

  my @shuffled = shuffle(@$list);

  if (scalar(@shuffled) < $n) {
    return @shuffled;
  }
  else {
    return @shuffled[0 .. $n-1];
  }
}

# input: %args: keys: file
# output: $contents
sub readFile {
  my %args = @_; 

  open(my $FH, '<', $args{file}) or die "Error, could not read $args{file}: $!";
  my $contents = do { local $/; <$FH> }; # localizing $? wipes the line separator char, so <> gets it all at once.
  close $FH;

  return $contents;
}

