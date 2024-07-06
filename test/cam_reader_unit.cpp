#include <gtest/gtest.h>

#include "../src/cam_reader.hpp"



// Test: Cas de base avec une séquence complète "b...e"
TEST(CamReader, ExtractLastCompleteSequenceBasicSequence) {
    const char* input = "abcde";
    std::string expected = "bcde";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Séquence avec plusieurs 'b' et 'e', prendre la dernière séquence complète
TEST(CamReader, ExtractLastCompleteSequenceMultipleSequences) {
    const char* input = "abcde fghbijklmne";
    std::string expected = "bijklmne";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Pas de 'b' dans la chaîne
TEST(CamReader, ExtractLastCompleteSequenceNoBCharacter) {
    const char* input = "acde";
    std::string expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Pas de 'e' dans la chaîne
TEST(CamReader, ExtractLastCompleteSequenceNoECharacter) {
    const char* input = "abcdf";
    std::string expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: 'b' après le dernier 'e'
TEST(CamReader, ExtractLastCompleteSequenceBAfterLastE) {
    const char* input = "abcdeb";
    std::string expected = "bcde";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Chaîne vide
TEST(CamReader, ExtractLastCompleteSequenceEmptyString) {
    const char* input = "";
    std::string expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Chaîne sans 'b' et 'e'
TEST(CamReader, ExtractLastCompleteSequenceNoBandECharacters) {
    const char* input = "xyz";
    std::string expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Chaîne contenant uniquement 'b' et 'e'
TEST(CamReader, ExtractLastCompleteSequenceOnlyBandECharacters) {
    const char* input = "bebebeb";
    std::string expected = "beb";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: 'b' au début et 'e' à la fin
TEST(CamReader, ExtractLastCompleteSequenceBAtStartEAtEnd) {
    const char* input = "b...e";
    std::string expected = "b...e";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}